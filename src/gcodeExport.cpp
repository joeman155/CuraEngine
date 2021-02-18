//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <assert.h>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <algorithm>

#include "Application.h" //To send layer view data.
#include "ExtruderTrain.h"
#include "gcodeExport.h"
#include "PrintFeature.h"
#include "RetractionConfig.h"
#include "Slice.h"
#include "communication/Communication.h" //To send layer view data.
#include "settings/types/LayerIndex.h"
#include "settings/types/Ratio.h"
#include "utils/Date.h"
#include "utils/logoutput.h"
#include "utils/string.h" // MMtoStream, PrecisionedDouble
#include "WipeScriptConfig.h"

namespace cura {

std::string transliterate(const std::string& text)
{
    // For now, just replace all non-ascii characters with '?'.
    // This function can be expaned if we need more complex transliteration.
    std::ostringstream stream;
    for (const char& c : text)
    {
        stream << static_cast<char>((c >= 0) ? c : '?');
    }
    return stream.str();
}

GCodeExport::GCodeExport()
: output_stream(&std::cout)
, currentPosition(0,0,MM2INT(20))
, layer_nr(0)
, relative_extrusion(false)
{
    *output_stream << std::fixed;

    premove_extrude    = 0;    // Amount of material extruded in PRE-MOVE. This is also used to identify is pre-extrude
                               // has already occured and is used to work out how much LESS to extrude at the end.
    Fxy                = 0;    // A value of zero means we haven't yet calculated it.
    e_speed_step       = 0;    // The speed step we are on
    m_speed_step       = 0;    // The speed step we are on
    speed_step_count   = 0;    // Number of speed steps we have
    step_min_speed     = 2;    // Minimum speed in mm/sec
    step_increment     = 2;    // How much to increment speed by each step (mm/sec)
    step_direction     = 1;    // 1 = Going up... 
    travel_dist_since_step = 0; // 0 means we are ready to do ramp up or down. A value less than extruderDistance indicates we haven't done enough yet...to allow another ramp up or down.
    current_e_value = 0;
    current_e_value_abs = 0;
    current_extruder = 0;
    current_fan_speed = -1;

    total_print_times = std::vector<Duration>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);

    currentSpeed = 1;
    current_print_acceleration = -1;
    current_travel_acceleration = -1;
    current_jerk = -1;

    is_z_hopped = 0;
    setFlavor(EGCodeFlavor::MARLIN);
    initial_bed_temp = 0;
    build_volume_temperature = 0;
    machine_heated_build_volume = false;

    fan_number = 0;
    use_extruder_offset_to_offset_coords = false;
    machine_name = "";
    machine_buildplate_type = "";
    relative_extrusion = false;
    new_line = "\n";

    total_bounding_box = AABB3D();
}

GCodeExport::~GCodeExport()
{
}

void GCodeExport::preSetup(const size_t start_extruder)
{
    current_extruder = start_extruder;

    const Scene& scene = Application::getInstance().current_slice->scene;
    std::vector<MeshGroup>::iterator mesh_group = scene.current_mesh_group;
    setFlavor(mesh_group->settings.get<EGCodeFlavor>("machine_gcode_flavor"));
    use_extruder_offset_to_offset_coords = mesh_group->settings.get<bool>("machine_use_extruder_offset_to_offset_coords");
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();

    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = scene.extruders[extruder_nr];
        setFilamentDiameter(extruder_nr, train.settings.get<coord_t>("material_diameter"));

        extruder_attr[extruder_nr].last_retraction_prime_speed = train.settings.get<Velocity>("retraction_prime_speed"); // the alternative would be switch_extruder_prime_speed, but dual extrusion might not even be configured...
        extruder_attr[extruder_nr].fan_number = train.settings.get<size_t>("machine_extruder_cooling_fan_number");
    }

    machine_name = mesh_group->settings.get<std::string>("machine_name");
    machine_buildplate_type = mesh_group->settings.get<std::string>("machine_buildplate_type");

    relative_extrusion = mesh_group->settings.get<bool>("relative_extrusion");
    always_write_active_tool = mesh_group->settings.get<bool>("machine_always_write_active_tool");

    if (flavor == EGCodeFlavor::BFB)
    {
        new_line = "\r\n";
    }
    else 
    {
        new_line = "\n";
    }

    estimateCalculator.setFirmwareDefaults(mesh_group->settings);
}

void GCodeExport::setInitialAndBuildVolumeTemps(const unsigned int start_extruder_nr)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = scene.extruders[extruder_nr];

        const Temperature print_temp_0 = train.settings.get<Temperature>("material_print_temperature_layer_0");
        const Temperature print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.settings.get<Temperature>("material_print_temperature");
        const Temperature temp = (extruder_nr == start_extruder_nr)? print_temp_here : train.settings.get<Temperature>("material_standby_temperature");
        setInitialTemp(extruder_nr, temp);
    }

    initial_bed_temp = scene.current_mesh_group->settings.get<Temperature>("material_bed_temperature_layer_0");
    machine_heated_build_volume = scene.current_mesh_group->settings.get<bool>("machine_heated_build_volume");
    build_volume_temperature = machine_heated_build_volume ? scene.current_mesh_group->settings.get<Temperature>("build_volume_temperature") : Temperature(0);
}

void GCodeExport::setInitialTemp(int extruder_nr, double temp)
{
    extruder_attr[extruder_nr].initial_temp = temp;
    if (flavor == EGCodeFlavor::GRIFFIN || flavor == EGCodeFlavor::ULTIGCODE)
    {
        extruder_attr[extruder_nr].currentTemperature = temp;
    }
}

const std::string GCodeExport::flavorToString(const EGCodeFlavor& flavor) const
{
    switch (flavor)
    {
        case EGCodeFlavor::BFB:
            return "BFB";
        case EGCodeFlavor::MACH3:
            return "Mach3";
        case EGCodeFlavor::MAKERBOT:
            return "Makerbot";
        case EGCodeFlavor::ULTIGCODE:
            return "UltiGCode";
        case EGCodeFlavor::MARLIN_VOLUMATRIC:
            return "Marlin(Volumetric)";
        case EGCodeFlavor::GRIFFIN:
            return "Griffin";
        case EGCodeFlavor::REPETIER:
            return "Repetier";
        case EGCodeFlavor::REPRAP:
            return "RepRap";
        case EGCodeFlavor::MARLIN:
        default:
            return "Marlin";
    }
}

std::string GCodeExport::getFileHeader(const std::vector<bool>& extruder_is_used, const Duration* print_time, const std::vector<double>& filament_used, const std::vector<std::string>& mat_ids)
{
    std::ostringstream prefix;

    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    switch (flavor)
    {
    case EGCodeFlavor::GRIFFIN:
        prefix << ";START_OF_HEADER" << new_line;
        prefix << ";HEADER_VERSION:0.1" << new_line;
        prefix << ";FLAVOR:" << flavorToString(flavor) << new_line;
        prefix << ";GENERATOR.NAME:Cura_SteamEngine" << new_line;
        prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
        prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
        prefix << ";TARGET_MACHINE.NAME:" << transliterate(machine_name) << new_line;

        for (size_t extr_nr = 0; extr_nr < extruder_count; extr_nr++)
        {
            if (!extruder_is_used[extr_nr])
            {
                continue;
            }
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:" << extruder_attr[extr_nr].initial_temp << new_line;
            if (filament_used.size() == extruder_count)
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << static_cast<int>(filament_used[extr_nr]) << new_line;
            }
            if (mat_ids.size() == extruder_count && mat_ids[extr_nr] != "")
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.GUID:" << mat_ids[extr_nr] << new_line;
            }
            const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extr_nr].settings;
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:" << extruder_settings.get<double>("machine_nozzle_size") << new_line;
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.NAME:" << extruder_settings.get<std::string>("machine_nozzle_id") << new_line;
        }
        prefix << ";BUILD_PLATE.TYPE:" << machine_buildplate_type << new_line;
        prefix << ";BUILD_PLATE.INITIAL_TEMPERATURE:" << initial_bed_temp << new_line;

        if (machine_heated_build_volume)
        {
            prefix << ";BUILD_VOLUME.TEMPERATURE:" << build_volume_temperature << new_line;
        }

        if (print_time)
        {
            prefix << ";PRINT.TIME:" << static_cast<int>(*print_time) << new_line;
        }

        prefix << ";PRINT.GROUPS:" << Application::getInstance().current_slice->scene.mesh_groups.size() << new_line;

        if (total_bounding_box.min.x > total_bounding_box.max.x) //We haven't encountered any movement (yet). This probably means we're command-line slicing.
        {
            //Put some small default in there.
            total_bounding_box.min = Point3(0, 0, 0);
            total_bounding_box.max = Point3(10, 10, 10);
        }
        prefix << ";PRINT.SIZE.MIN.X:" << INT2MM(total_bounding_box.min.x) << new_line;
        prefix << ";PRINT.SIZE.MIN.Y:" << INT2MM(total_bounding_box.min.y) << new_line;
        prefix << ";PRINT.SIZE.MIN.Z:" << INT2MM(total_bounding_box.min.z) << new_line;
        prefix << ";PRINT.SIZE.MAX.X:" << INT2MM(total_bounding_box.max.x) << new_line;
        prefix << ";PRINT.SIZE.MAX.Y:" << INT2MM(total_bounding_box.max.y) << new_line;
        prefix << ";PRINT.SIZE.MAX.Z:" << INT2MM(total_bounding_box.max.z) << new_line;
        prefix << ";END_OF_HEADER" << new_line;
        break;
    default:
        prefix << ";FLAVOR:" << flavorToString(flavor) << new_line;
        prefix << ";TIME:" << ((print_time)? static_cast<int>(*print_time) : 6666) << new_line;
        if (flavor == EGCodeFlavor::ULTIGCODE)
        {
            prefix << ";MATERIAL:" << ((filament_used.size() >= 1)? static_cast<int>(filament_used[0]) : 6666) << new_line;
            prefix << ";MATERIAL2:" << ((filament_used.size() >= 2)? static_cast<int>(filament_used[1]) : 0) << new_line;

            prefix << ";NOZZLE_DIAMETER:" << Application::getInstance().current_slice->scene.extruders[0].settings.get<double>("machine_nozzle_size") << new_line;
        }
        else if (flavor == EGCodeFlavor::REPRAP || flavor == EGCodeFlavor::MARLIN || flavor == EGCodeFlavor::MARLIN_VOLUMATRIC)
        {
            prefix << ";Filament used: ";
            if (filament_used.size() > 0)
            {
                for (unsigned i = 0; i < filament_used.size(); ++i)
                {
                    if (i > 0)
                    {
                        prefix << ", ";
                    }
                    if (flavor != EGCodeFlavor::MARLIN_VOLUMATRIC)
                    {
                        prefix << filament_used[i] / (1000 * extruder_attr[i].filament_area) << "m";
                    }
                    else //Use volumetric filament used.
                    {
                        prefix << filament_used[i] << "mm3";
                    }
                }
            }
            else
            {
                prefix << "0m";
            }
            prefix << new_line;
            prefix << ";Layer height: " << Application::getInstance().current_slice->scene.current_mesh_group->settings.get<double>("layer_height") << new_line;
        }
        prefix << ";MINX:" << INT2MM(total_bounding_box.min.x) << new_line;
        prefix << ";MINY:" << INT2MM(total_bounding_box.min.y) << new_line;
        prefix << ";MINZ:" << INT2MM(total_bounding_box.min.z) << new_line;
        prefix << ";MAXX:" << INT2MM(total_bounding_box.max.x) << new_line;
        prefix << ";MAXY:" << INT2MM(total_bounding_box.max.y) << new_line;
        prefix << ";MAXZ:" << INT2MM(total_bounding_box.max.z) << new_line;
    }

    return prefix.str();
}


void GCodeExport::setLayerNr(unsigned int layer_nr_) {
    layer_nr = layer_nr_;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
    output_stream = stream;
    *output_stream << std::fixed;
}

bool GCodeExport::getExtruderIsUsed(const int extruder_nr) const
{
    assert(extruder_nr >= 0);
    assert(extruder_nr < MAX_EXTRUDERS);
    return extruder_attr[extruder_nr].is_used;
}

Point GCodeExport::getGcodePos(const coord_t x, const coord_t y, const int extruder_train) const
{
    if (use_extruder_offset_to_offset_coords)
    {
        const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extruder_train].settings;
        return Point(x - extruder_settings.get<coord_t>("machine_nozzle_offset_x"), y - extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
    }
    else
    {
        return Point(x, y);
    }
}


void GCodeExport::setFlavor(EGCodeFlavor flavor)
{
    this->flavor = flavor;
    if (flavor == EGCodeFlavor::MACH3)
    {
        for(int n=0; n<MAX_EXTRUDERS; n++)
        {
            extruder_attr[n].extruderCharacter = 'A' + n;
        }
    }
    else
    {
        for(int n=0; n<MAX_EXTRUDERS; n++)
        {
            extruder_attr[n].extruderCharacter = 'E';
        }
    }
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::MARLIN_VOLUMATRIC)
    {
        is_volumetric = true;
    }
    else
    {
        is_volumetric = false;
    }
}

EGCodeFlavor GCodeExport::getFlavor() const
{
    return flavor;
}

void GCodeExport::setZ(int z)
{
    current_layer_z = z;
}

void GCodeExport::setFlowRateExtrusionSettings(double max_extrusion_offset, double extrusion_offset_factor)
{
    this->max_extrusion_offset = max_extrusion_offset;
    this->extrusion_offset_factor = extrusion_offset_factor;
}

Point3 GCodeExport::getPosition() const
{
    return currentPosition;
}
Point GCodeExport::getPositionXY() const
{
    return Point(currentPosition.x, currentPosition.y);
}

int GCodeExport::getPositionZ() const
{
    return currentPosition.z;
}

int GCodeExport::getExtruderNr() const
{
    return current_extruder;
}

void GCodeExport::setFilamentDiameter(const size_t extruder, const coord_t diameter)
{
    const double r = INT2MM(diameter) / 2.0;
    const double area = M_PI * r * r;
    extruder_attr[extruder].filament_area = area;
}

double GCodeExport::getCurrentExtrudedVolume() const
{
    double extrusion_amount = current_e_value;
    const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if (!extruder_settings.get<bool>("machine_firmware_retract"))
    { // no E values are changed to perform a retraction
        extrusion_amount -= extruder_attr[current_extruder].retraction_e_amount_at_e_start; // subtract the increment in E which was used for the first unretraction instead of extrusion
        extrusion_amount += extruder_attr[current_extruder].retraction_e_amount_current; // add the decrement in E which the filament is behind on extrusion due to the last retraction
    }
    if (is_volumetric)
    {
        return extrusion_amount;
    }
    else
    {
        return extrusion_amount * extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::eToMm(double e)
{
    if (is_volumetric)
    {
        return e / extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return e;
    }
}

double GCodeExport::mm3ToE(double mm3)
{
    if (is_volumetric)
    {
        return mm3;
    }
    else
    {
        return mm3 / extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::mmToE(double mm)
{
    if (is_volumetric)
    {
        return mm * extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return mm;
    }
}

double GCodeExport::eToMm3(double e, size_t extruder)
{
    if (is_volumetric)
    {
        return e;
    }
    else
    {
        return e * extruder_attr[extruder].filament_area;
    }
}

double GCodeExport::getTotalFilamentUsed(size_t extruder_nr)
{
    if (extruder_nr == current_extruder)
        return extruder_attr[extruder_nr].totalFilament + getCurrentExtrudedVolume();
    return extruder_attr[extruder_nr].totalFilament;
}

std::vector<Duration> GCodeExport::getTotalPrintTimePerFeature()
{
    return total_print_times;
}

double GCodeExport::getSumTotalPrintTimes()
{
    double sum = 0.0;
    for(double item : getTotalPrintTimePerFeature())
    {
        sum += item;
    }
    return sum;
}

void GCodeExport::resetTotalPrintTimeAndFilament()
{
    for(size_t i = 0; i < total_print_times.size(); i++)
    {
        total_print_times[i] = 0.0;
    }
    for(unsigned int e=0; e<MAX_EXTRUDERS; e++)
    {
        extruder_attr[e].totalFilament = 0.0;
        extruder_attr[e].currentTemperature = 0;
        extruder_attr[e].waited_for_temperature = false;
    }
    current_e_value = 0.0;
    estimateCalculator.reset();
}

void GCodeExport::updateTotalPrintTime()
{
    std::vector<Duration> estimates = estimateCalculator.calculate();
    for(size_t i = 0; i < estimates.size(); i++)
    {
        total_print_times[i] += estimates[i];
    }
    estimateCalculator.reset();
    writeTimeComment(getSumTotalPrintTimes());
}

void GCodeExport::writeComment(const std::string& unsanitized_comment)
{
    const std::string comment = transliterate(unsanitized_comment);

    *output_stream << ";";
    for (unsigned int i = 0; i < comment.length(); i++)
    {
        if (comment[i] == '\n')
        {
            *output_stream << new_line << ";";
        }
        else
        {
            *output_stream << comment[i];
        }
    }
    *output_stream << new_line;
}

void GCodeExport::writeTimeComment(const Duration time)
{
    *output_stream << ";TIME_ELAPSED:" << time << new_line;
}

void GCodeExport::writeTypeComment(const PrintFeatureType& type)
{
    switch (type)
    {
        case PrintFeatureType::OuterWall:
            *output_stream << ";TYPE:WALL-OUTER" << new_line;
            break;
        case PrintFeatureType::InnerWall:
            *output_stream << ";TYPE:WALL-INNER" << new_line;
            break;
        case PrintFeatureType::Skin:
            *output_stream << ";TYPE:SKIN" << new_line;
            break;
        case PrintFeatureType::Support:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::SkirtBrim:
            *output_stream << ";TYPE:SKIRT" << new_line;
            break;
        case PrintFeatureType::Infill:
            *output_stream << ";TYPE:FILL" << new_line;
            break;
        case PrintFeatureType::SupportInfill:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::SupportInterface:
            *output_stream << ";TYPE:SUPPORT-INTERFACE" << new_line;
            break;
        case PrintFeatureType::PrimeTower:
            *output_stream << ";TYPE:PRIME-TOWER" << new_line;
            break;
        case PrintFeatureType::MoveCombing:
        case PrintFeatureType::MoveRetraction:
        case PrintFeatureType::NoneType:
        case PrintFeatureType::NumPrintFeatureTypes:
            // do nothing
            break;
    }
}


void GCodeExport::writeLayerComment(const LayerIndex layer_nr)
{
    *output_stream << ";LAYER:" << layer_nr << new_line;
}

void GCodeExport::writeLayerCountComment(const size_t layer_count)
{
    *output_stream << ";LAYER_COUNT:" << layer_count << new_line;
}

void GCodeExport::writeLine(const char* line)
{
    *output_stream << line << new_line;
}

void GCodeExport::writeExtrusionMode(bool set_relative_extrusion_mode)
{
    if (set_relative_extrusion_mode)
    {
        *output_stream << "M83 ;relative extrusion mode" << new_line;
    }
    else
    {
        *output_stream << "M82 ;absolute extrusion mode" << new_line;
    }
}

void GCodeExport::resetExtrusionValue()
{
    if (!relative_extrusion)
    {
        *output_stream << "G92 " << extruder_attr[current_extruder].extruderCharacter << "0" << new_line;
    }
    double current_extruded_volume = getCurrentExtrudedVolume();
    extruder_attr[current_extruder].totalFilament += current_extruded_volume;
    for (double& extruded_volume_at_retraction : extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions)
    { // update the extruded_volume_at_previous_n_retractions only of the current extruder, since other extruders don't extrude the current volume
        extruded_volume_at_retraction -= current_extruded_volume;
    }
    current_e_value = 0.0;
    extruder_attr[current_extruder].retraction_e_amount_at_e_start = extruder_attr[current_extruder].retraction_e_amount_current;
}

void GCodeExport::writeDelay(const Duration& time_amount)
{
    *output_stream << "G4 P" << int(time_amount * 1000) << new_line;
    estimateCalculator.addTime(time_amount);
}

void GCodeExport::writeTravel(const Point& p, const Velocity& speed)
{
    writeTravel(Point3(p.X, p.Y, current_layer_z), speed);
}
void GCodeExport::writeExtrusion(const Point& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset, int next_distance_remaining)
{
    writeExtrusion(Point3(p.X, p.Y, current_layer_z), speed, extrusion_mm3_per_mm, feature, update_extrusion_offset, next_distance_remaining);
}

void GCodeExport::writeTravel(const Point3& p, const Velocity& speed)
{
    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x, p.y, p.z + is_z_hopped, speed, 0.0, PrintFeatureType::MoveCombing);
        return;
    }
    writeTravel(p.x, p.y, p.z + is_z_hopped, speed);
}

void GCodeExport::writeExtrusion(const Point3& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset, int next_distance_remaining)
{
    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature);
        return;
    }
    writeExtrusion(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature, update_extrusion_offset, next_distance_remaining);
}

void GCodeExport::writeMoveBFB(const int x, const int y, const int z, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature)
{
    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }
    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    Point gcode_pos = getGcodePos(x,y, current_extruder);

    //For Bits From Bytes machines, we need to handle this completely differently. As they do not use E values but RPM values.
    float fspeed = speed * 60;
    float rpm = extrusion_per_mm * speed * 60;
    const float mm_per_rpm = 4.0; //All BFB machines have 4mm per RPM extrusion.
    rpm /= mm_per_rpm;
    if (rpm > 0)
    {
        if (extruder_attr[current_extruder].retraction_e_amount_current)
        {
            if (currentSpeed != double(rpm))
            {
                //fprintf(f, "; %f e-per-mm %d mm-width %d mm/s\n", extrusion_per_mm, lineWidth, speed);
                //fprintf(f, "M108 S%0.1f\r\n", rpm);
                *output_stream << "M108 S" << PrecisionedDouble{1, rpm} << new_line;
                currentSpeed = double(rpm);
            }
            //Add M101 or M201 to enable the proper extruder.
            *output_stream << "M" << int((current_extruder + 1) * 100 + 1) << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
        }
        //Fix the speed by the actual RPM we are asking, because of rounding errors we cannot get all RPM values, but we have a lot more resolution in the feedrate value.
        // (Trick copied from KISSlicer, thanks Jonathan)
        fspeed *= (rpm / (roundf(rpm * 100) / 100));

        //Increase the extrusion amount to calculate the amount of filament used.
        Point3 diff = Point3(x,y,z) - getPosition();
        
        current_e_value += extrusion_per_mm * diff.vSizeMM();
        current_e_value_abs += extrusion_per_mm * diff.vSizeMM();
    }
    else
    {
        //If we are not extruding, check if we still need to disable the extruder. This causes a retraction due to auto-retraction.
        if (!extruder_attr[current_extruder].retraction_e_amount_current)
        {
            *output_stream << "M103" << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 1.0; // 1.0 used as stub; BFB doesn't use the actual retraction amount; it performs retraction on the firmware automatically
        }
    }
    *output_stream << "G1 X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y} << " Z" << MMtoStream{z};
    *output_stream << " F" << PrecisionedDouble{1, fspeed} << new_line;
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), speed, feature);
}

void GCodeExport::writeTravel(const coord_t& x, const coord_t& y, const coord_t& z, const Velocity& speed)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
    {
        return;
    }

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
#endif //ASSERT_INSANE_OUTPUT


    const PrintFeatureType travel_move_type = extruder_attr[current_extruder].retraction_e_amount_current ? PrintFeatureType::MoveRetraction : PrintFeatureType::MoveCombing;
    const int display_width = extruder_attr[current_extruder].retraction_e_amount_current ? MM2INT(0.2) : MM2INT(0.1);
    const double layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<double>("layer_height");
    Application::getInstance().communication->sendLineTo(travel_move_type, Point(x, y), display_width, layer_height, speed);

    *output_stream << "G0";
    writeFXYZE(speed, x, y, z, current_e_value, travel_move_type);

}



// JOE2
void GCodeExport::writeExtrusion(const int x, const int y, const int z, const Velocity& speed, const double extrusion_mm3_per_mm, const PrintFeatureType& feature, const bool update_extrusion_offset, int next_distance_remaining)
{

    // If current position == new position....no point creating a G-Code
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
    {
        return;
    }

    // Get Dispenser ratio and extruder_latency for fine tuning of timing....for tuning of Extruder speed.
    const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    const Ratio dispenser_ratio = extruder_settings.get<Ratio>("dispenser_ratio");
    const Ratio extruder_latency = extruder_settings.get<Ratio>("extruder_latency");
    
   

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
    assert(extrusion_mm3_per_mm >= 0.0);
#endif //ASSERT_INSANE_OUTPUT
#ifdef DEBUG
    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }

    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    if (extrusion_mm3_per_mm < 0.0)
    {
        logWarning("Warning! Negative extrusion move!\n");
    }
#endif

    const double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    if (is_z_hopped > 0)
    {
        writeZhopEnd();
    }

    const Point3 diff = Point3(x,y,z) - currentPosition;
    const double diff_length = diff.vSizeMM();

    writeUnretractionAndPrime();

    //flow rate compensation
    double extrusion_offset = 0;
    if(diff_length)
    {
        extrusion_offset = speed * extrusion_mm3_per_mm * extrusion_offset_factor;
        if (extrusion_offset > max_extrusion_offset)
        {
            extrusion_offset = max_extrusion_offset;
        }
    }
    // write new value of extrusion_offset, which will be remembered.
    if (update_extrusion_offset && (extrusion_offset != current_e_offset))
    {
        current_e_offset = extrusion_offset;
        *output_stream << ";FLOW_RATE_COMPENSATED_OFFSET = " << current_e_offset << new_line;
    }

    extruder_attr[current_extruder].last_e_value_after_wipe += extrusion_per_mm * diff_length;
    const double new_e_value = current_e_value + extrusion_per_mm * diff_length * dispenser_ratio;



  
   


   // Some of the printing is done as a polygon (path) and consists of multiple moves, extrusions all
   // as one. It makes sense to try and "string" these together in a non-stop extrusion multi-path move.
   // rather than as individual MOVES - which means lots of WAITS - longer print times.
   //
   // extruder_latency > 0                 - if we have no extruder_latency, (extruder_latency = 0), then we wouldn't want to run this code
   // new_e_value - current_e_vale > 0     - IF nothing to extrude... pointless.
   if (extruder_latency > 0 && new_e_value - current_e_value > 0) 
   {







      // Current Position
      Point3 cp = currentPosition;  // Record Current Position because currentPosition is overwritten

      // Get the change in E required
      double e_delta = new_e_value - current_e_value;

      // Calculate effective X/Y speed.
      Fxy = effectiveSpeed (cp, x, y, e_delta, speed);

      // Time to do the move
      double t0 = diff_length / Fxy;    // Time in seconds

      // TOTAL distance for these connected moves. 
      double total_distance_remaining = INT2MM(next_distance_remaining) + diff_length;         
      double total_time_remaining = total_distance_remaining / Fxy; // Assumes instant acceleration...

      // Distance the extruder moves in the time that it takes for the powder to fall and start hitting the plate.
      double extruder_distance = Fxy * extruder_latency;   

      // Calculate speed at which we need to EXTRUDE this material
      double despeed = e_delta / t0; // mm/second

// *output_stream << "; x,y,z = " << x << ", " << y << ", " << z << new_line;
// *output_stream << "; E_delta: " <<  e_delta << new_line;
// *output_stream << "; new_e_value: " << new_e_value << new_line;
// *output_stream << "; diff_length (First Move - mm): " << diff_length << new_line;
// *output_stream << "; Fxy (Effective XY Speed - mm/sec): " << Fxy << new_line;
// *output_stream << "; total_distance_remaining (Whole Path - mm): " << total_distance_remaining << new_line;
// *output_stream << "; total_time_remaining (whole Path - seconds): " << total_time_remaining << new_line;
// *output_stream << "; t0 (Time in second to do FIRST move): " << t0 << new_line;
// *output_stream << "; despeed (mm/sec of extrusion): " << despeed << new_line;
//  *output_stream << "; extruder_distance (Distance in mm the extruder moves (x,y) during complete prime): " << extruder_distance << new_line;


      // See if we need to start extrusion... WE ALWAYS need to extrude ONLY before starting a move
      // UNLESS extrusion has already started.
      if (premove_extrude == 0) 
      {
 
         //JOE3
         // FIRST WE DO SOME CALCS TO SET US UP FOR RAMP up and RAMP DOWN
         // NOTE.... this ramp-up and ramp-down does NOT affect speed of the initial extrude.
         speed_step_count = 0;
         if (speed > step_min_speed) {
            speed_step_count = round((speed - step_min_speed) / step_increment);
         }
         e_speed_step = speed_step_count;     // No steps taken yet
         m_speed_step = e_speed_step;
         step_direction = 1;  // We are ramping up.



         // Include some useful info on type of path
         if (next_distance_remaining > 0)  {
             *output_stream << "; Multipath extrusion of distance: " << total_distance_remaining << " mm" << new_line;
         } else if (next_distance_remaining <=0) {
            *output_stream << "; SINGLE line extrusion: " << diff_length << " mm" << new_line;
         } else {
            *output_stream << "; OTHER line extrusion of distance: " << INT2MM(next_distance_remaining) << " mm" << new_line;
         }


         // The premove-extrude ALWAYS has to start at the slowest speed
         double pfactor = (speed - e_speed_step * step_increment) / speed;


         // Determine amount to extrude
         // Here we manage the case where...TOTAL Time to extrude > extruder latency
         //
         // Think for a second....
         // This includes scenarion were for THIS move, the amount to extrude may be small... 
         // because it is a ShORT distance... It might be a LOT of small moves...which has a 
         // total time remaining > extruder latency
         //
         if (total_time_remaining > extruder_latency) 
         {
            // Calculate the NEW E-value... to just extrude
            premove_extrude = despeed * extruder_latency * pfactor; // More intuituve form... same result
            double ext_move = current_e_value + premove_extrude;

            *output_stream << "G1";
            writeFE(Velocity(pfactor * despeed), ext_move, feature);
         }
         else
         {
            // Calculate amount to extrude on latter moves
            double e3_next = e_delta * (INT2MM(next_distance_remaining) / diff_length);

            // Grand total on amount to extrude
            premove_extrude = pfactor * (e_delta + e3_next);

            // First Move (E only, no x, y)
            double e1 = current_e_value + premove_extrude;
 
            // Calculate amount of time spent extruding this very small amount of powder
            double t_spent =  premove_extrude / despeed;

            // Calculate amount of time to wait for powder to hit the plate.
            double t_remaining = extruder_latency - t_spent;

            *output_stream << "G1";
            writeFE(Velocity(pfactor * despeed), e1, feature);
 
            *output_stream << "; Partial extrude" << new_line;

            // Generate G-Code command to wait for powder to hit plate.
            // Multiple by 1000 to convert seconds to milliseconds
            int t_remaining_ms = int(t_remaining* 1000);
            *output_stream << "G4 P" << t_remaining_ms << new_line;
         }

      }  // End of test "premove_extrude == 0"


     double dist_remaining = 0;

     // See how we want to proceed with the G-Codes
     if (INT2MM(next_distance_remaining) > extruder_distance)
     {
        // The NEXT move after this is greater than extruder_latency, which means we don't have to 
        // worry about finishing the whole "move" this time. THIS IS EASY CASE

        // Second, 3rd.... Move (E and x and y)

        // We just want to extrude the amount to keep it fully primed. IT is already primed...we are just
        // topping up...as it falls out the bottom


        double e_change = e_delta; // We know we have lots more to extrude in latter moves... so we simply 
                                   // want to top up what is used in this move.
        double e3 = current_e_value + e_change;

    
        dist_remaining = diff_length;  // Track how much further we move INSIDE this move.
        dist_remaining = rampUpDownExtrude(dist_remaining, next_distance_remaining, total_distance_remaining, extruder_distance, speed,
                                           x, y, z, e3, feature);


     } 
     else if (next_distance_remaining > 0 && INT2MM(next_distance_remaining) < extruder_distance && total_distance_remaining > extruder_distance)
     {

        // Situation: Some movement(s) after this, (but less than extruder length). TOTAL distance to move requires SOME extrusion.
        // 
        // Considering JUST this move, possible scenarios are:-
        //       a. This move < extruder_distance
        //       b. This move > extruder_distance



        // NOTE: There could be several more moves after this...but total movement of all these 
        //       moves is under extruder_distance
        //


        double e_change = e_delta; // We know we have lots more to extrude in latter moves... so we simply 
                                   // want to top up what is used in this move.
        double e3 = current_e_value + e_change;

    
        dist_remaining = diff_length;  // Track how much further we move INSIDE this move.
        dist_remaining = rampUpDownExtrude(dist_remaining, next_distance_remaining, total_distance_remaining, extruder_distance, speed,
                                           x, y, z, e3, feature);

     }
     else  
     {
        // Possible scenarios are:-
        // 1. No movement after this move.
        //       a. This move < extruder_distance
        //       b. This move > extruder_distance
        // 
        // 2. More movements after this... and TOTAL length to move < extrusion_distance. i.e. no more extrusion require.
        // 

        double e_change = e_delta; // We know we have lots more to extrude in latter moves... so we simply 
                                   // want to top up what is used in this move.
        double e3 = current_e_value + e_change;

        dist_remaining = diff_length;  // Track how much further we move INSIDE this move.
        dist_remaining = rampUpDownExtrude(dist_remaining, next_distance_remaining, total_distance_remaining, extruder_distance, speed,
                                           x, y, z, e3, feature);


     }



   } 
   else 
   {
     // Get to this point if there is nothig to extrude, or if the latency setting is zero. i.e. STANDARD FUNCTIONALITY
     *output_stream << "G1";
     writeFXYZE(speed, x, y, z, new_e_value, feature);
    }




log("\n\n\n");




}

// JOE
void GCodeExport::writeFXYZE(const Velocity& speed, const int x, const int y, const int z, const double e, const PrintFeatureType& feature)
{

    const double max_extrusion = 200;
    const Point3 diff = Point3(x,y,z) - currentPosition;
    double local_e_value = current_e_value;
    const double total_extrusion = e - local_e_value;

    /* The firmware that is installed by default on the Creality Ender5 restricts MAX extrusion in ONE command to 200 */
    /* If the code above needs to perform a legitmate extrusion greater than 200, we split it up into smaller extusions. */
    if (total_extrusion > max_extrusion) {
        int px;              // PART x distance we move the extruder, all while we keep extruded amount <= 200
        int py;              // PART y distance we move the extruder, all while we keep extruded amount <= 200
        double pe;           // PART e - how much we extrude this time
        double m;            // Slope the path of extruder across plate
        int travel_remaining;  // How far of the 'total_travel' we have left
        int dist;            // Used to hold value to move extruder
        double extrusion_remaining;  // How much extrusion remaining to do.
        const int total_travel = diff.vSize();
        double direction = 1.0; // default to this.

        // Calculate slope of path (the direction of travel)
        if (y == currentPosition.y) {
           m = 0;
           if (x < currentPosition.x) {
              direction = -1.0;
           }
        } else if (x == currentPosition.x) {
           m = 999999999; // Infinite
           if (y < currentPosition.y) {
              direction = -1.0;
           }
        } else {
           m = ((double)y - (double)currentPosition.y) / ((double)x - (double)currentPosition.x);
        }


        // Work out the total extrusions amounts left... as a starting point
        travel_remaining      = total_travel;
        extrusion_remaining = total_extrusion;

        // while we have extruding left... EXTRUDE!!
        bool finished = false;
        px = currentPosition.x;
        py = currentPosition.y;
        while (finished == false) {
      
           // Calculate the Max distance we can travel (over which the max extrusion will take place)
           if (extrusion_remaining > max_extrusion) {
              dist =  travel_remaining *  (max_extrusion / extrusion_remaining); 
           } else {
              dist = travel_remaining;
              finished = true;
           }
        

           // Calculate intermediate X and Y position. This is complicated by real-life horizontal and vertial paths
           if (m == 0) {
              // Flat line, only px changes
              px = px + direction * dist;
           } else if (m == 999999999) {
              // Vertical line, only py changes
              py = py + direction * dist;
           } else {
              px = px + dist / sqrt( 1 + m * m);
              py = py + m * (dist / sqrt(1 + m * m));
           }


           pe = total_extrusion * dist / total_travel;
           local_e_value = pe + local_e_value;
       

           // If still going extruding (and not the first one)...then prefix with G1
           if (travel_remaining < total_travel) {
              *output_stream << "G1";
            }
           GCodeExport::writeFXYZEpart(speed, px, py, z, local_e_value, feature);

           // Deduct the distance travelled and extruded material from counters
           travel_remaining = travel_remaining - dist;
           extrusion_remaining = extrusion_remaining - pe;
        }

        // Print out something in GCode, so HUMAN knows what was done here...
        *output_stream << "; Delta E > " << max_extrusion << " ...splitting it up. This is to accomodate Ender5 Firmware limit." << new_line;
        *output_stream << new_line << "; Total Travel: " << total_travel << new_line;
        *output_stream << "; Extrusion Remaining: " << total_extrusion << new_line;
        *output_stream << "; Slope: " << m << new_line << new_line;


        *output_stream << ";x = " << x <<new_line << new_line;



    } else {
      // Under the limit...so generate code as usual.
      GCodeExport::writeFXYZEpart(speed, x, y, z, e, feature);
    }
}



/*
 * Calculate the distance the extruder moves for 
 * - the standard distance it moves - normal speed
 * - the speed_step that the x,y is on
 *
 * We work this out by getting pfactor, which indicates how much slower the extruder moves...and
 * just multiplies this by the standard extruder_distance.
 *
 *
*/
double GCodeExport::getExtrudeDistance (double extruder_distance, int speed_step, const Velocity &speed)
{
   double pfactor;

   pfactor = (speed - speed_step * step_increment) / speed;

   double distance = extruder_distance * pfactor;

   return distance;
}




double GCodeExport::rampUpDownExtrude(double dist_remaining, int next_distance_remaining, double total_distance_remaining, double extruder_distance, 
                                      const Velocity &speed, const int x, const int y, const int z, const double e, const PrintFeatureType& feature)
{
   double new_dist_remaining = dist_remaining;


   double e_d;
   double e_delta;
   double distance;
   bool finished;

   Point3 cp1 = currentPosition;
   distance = INT2MM(sqrt ((x - cp1.x) * (x - cp1.x) + (y - cp1.y) * (y - cp1.y)));
   e_delta = e - current_e_value;
   e_d = e_delta / distance;    // Density of powder extrusion PER mm of distance...what we need to maintain on whole of
                                // extrusion.

   new_dist_remaining = rampUpExtrude(new_dist_remaining, next_distance_remaining, total_distance_remaining, extruder_distance, speed,
                                      x, y, z, e, feature, e_d, e_delta, &finished);




   // IF there is still some extruding...this might because rampUpExtrude didn't think it should...and that rampDownExtrude should
   if (finished == false) {
      new_dist_remaining = rampDownExtrude(new_dist_remaining, next_distance_remaining, extruder_distance, speed,
                                           x, y, z, e, feature, e_d, e_delta, &finished);
   }


   // IF STILL some extruding...this will be cause ramping down isn't quite the activity we need to do just yet.
   // So we just Extrude some more at the CURRENT speed.
   while (finished == false) {
      new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 1, e_d, 0, next_distance_remaining, &finished);
   }

   return new_dist_remaining;
}





double GCodeExport::rampUpExtrude(double dist_remaining, int next_distance_remaining, double total_distance_remaining, double extruder_distance, const Velocity &speed,
                                  const int x, const int y, const int z, const double e, const PrintFeatureType& feature, double e_d, double e_delta, bool *finished)
{
  int max_splits;               // Maximum # of steps we can do this move
  int extrude_splits;           // Number of steps we will actually do.
  double new_dist_remaining = dist_remaining;

  // Assume not finished
  *finished = false;

  if (step_direction != 1) {    // We are only considering cases where we are Ramping up. So exit
     return new_dist_remaining;
  }
  
  if (m_speed_step == 0) {      // Already ramped up....
     step_direction = -1;       // Any future ramping is DOWN... (though it might not be for a while)
     return new_dist_remaining;
  }
  
  if (total_distance_remaining <= rampDistance(speed_step_count, step_increment, extruder_distance, speed)) {
     // Already close to having to ramp down...so we don't try to ramp up.
     step_direction = -1;       // Any future ramping is DOWN ... and is likely to be for this move ... in latter code.
     return new_dist_remaining;
  }


  // If we are still through a ramp up or ramp down, we must first attempt to finish it.
  if (travel_dist_since_step > 0 && travel_dist_since_step < getExtrudeDistance (extruder_distance, m_speed_step, speed)) {
     new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 0, e_d, 0, next_distance_remaining, finished);
     if (*finished == true) {
        // We have finished off the move....so we exit. no point trying to split NOTHING INTO SOMETHING
        return new_dist_remaining;
     }
  }


  // IT is possible that we finish off a ramp up here... and we are at the end...and so we are not interested in any more ramping up
  if (m_speed_step == 0) {      // Already ramped up....
     step_direction = -1;       // Any future ramping is DOWN... (though it might not be for a while)
     return new_dist_remaining;
  }



  // Get maximum # of splits
  max_splits = maxSteps(step_direction, new_dist_remaining, speed_step_count, step_increment, extruder_distance, speed, m_speed_step);


  // IF we aren't breaking this into pieces, because the piece is TOO small...then we have the chance of ramping up ONE step here .. POTENTIALLY
  if (max_splits == 0) {
     *output_stream << "; Small move up: " << new_dist_remaining << new_line;
     // We only want to ramp up IF we have fully extruded one EXTRUDER full in the previous step.
     // We STILL may be able to ramp up ONE notch if the distance since last ramp is greater than the previous extruder_distance 
     //
     // It is important to note...that if  we do not try to COMMENCE ramp up part-way through a small step.
     if (e_speed_step == m_speed_step) {
        // Commence new RAMP UP.... as we aren't alrady in a ramp up ... and it has been ample distance since last one.
        e_speed_step--;
     } else {   // We continue to extrude at given speed. Nothing else to do here...
     }
     new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 0, e_d, 0, next_distance_remaining, finished);

     return new_dist_remaining;   // No point continuing on, because we have finished this "run"
  } 


  // Get number of steps we can do in this case.
  extrude_splits = std::min(m_speed_step, max_splits);
  *output_stream << "; RAMP UP - extrude_splits: " << extrude_splits << ", e_speed_step: " << e_speed_step << new_line;

  int initial_speed_step = e_speed_step;
  for (int step = e_speed_step - 1; step >= (initial_speed_step - extrude_splits); step--) {
     e_speed_step = step;
   
     // TODO - NEED TO CATER FOR SCENARIO WHERE WE FINISh OFF TRAVEL BEFORE WE DO THE SPLITS.
     new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 0, e_d, 0, next_distance_remaining, finished);


     // Do work - generate the line
     // TODO - Calculate the new extruder_distance..... since it is reduced based on pfactor
     // TODO - Calculate the new E value (ratio of extruder_distance to diff_length x e_delta)
     // TODO - Calculate the new speed (x by pfactor)
     // TODO - Generate new X,Y,Z points   (ratio of extruder_distance to diff_length x Each component)
     // TODO - Generate new G1 Code line

  
     // If we are getting close to the end...at the point where we need to start reversing speed... we exit and signal that this
     // is what we need to start doing...
     if (step_direction == 1 && INT2MM(next_distance_remaining) + new_dist_remaining < rampDistance(speed_step_count, step_increment, extruder_distance, speed) ) {
        step_direction = -1;
        break;
     }
  } 



             
  // If we are at m_speed_step == 0... we have done all the ramping up we can do. The only ramping 
  // to do now for this potential multi-path is to ramp down.
  if (m_speed_step == 0) {
     step_direction = -1;     // Next time we change speed...we are ramping down.
  }



  return new_dist_remaining;
}



double GCodeExport::rampDownExtrude(double dist_remaining, int next_distance_remaining, double extruder_distance, const Velocity &speed,
                                    const int x, const int y, const int z, const double e, const PrintFeatureType& feature, double e_d, double e_delta, bool *finished)
{
  int max_splits;                // Maximum # of steps we can do this move
  int extrude_splits;            // Number of steps we will actually do.
  double move_contingency = 3;   // Clearance we give us from the end, so when we finish ramp down we aren't RIGHT ON THE END.
  double new_dist_remaining = dist_remaining;

  // Assume not finished
  *finished = false;

  if (step_direction != -1) {
     // If we are not ready to ramp down...then return
     return new_dist_remaining;
  }
  

  if (INT2MM(next_distance_remaining) >= rampDistance(speed_step_count, step_increment, extruder_distance, speed)) {
     // Still a long way to go, so not time to ramp down.
     return new_dist_remaining;
  }

  // If we are still through a ramp up or ramp down, we must first attempt to finish it.
  if (travel_dist_since_step > 0 && travel_dist_since_step < getExtrudeDistance (extruder_distance, m_speed_step, speed)) {
     new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 0, e_d, 0, next_distance_remaining, finished);
     if (*finished == true) {
        // We have finished off the move....so we exit. no point trying to split NOTHING INTO SOMETHING
        return new_dist_remaining;
     }
  }



  // IT is possible that we finish off a ramp up here... and we are at the end...and so we are not interested in any more ramping up
  if (m_speed_step == speed_step_count) {      // Already ramped up....
     step_direction = 2;       //  No more ramp up or down for this PATH
     return new_dist_remaining;
  }


  // We want to deal with case where the next_remaining_distance is large, but not large enough for ramping down
  // and we have a LARGE distance to travel this move... and if we start ramping down STRAIGHTAWAY...we will spend
  // a significant amount of time at the SLOW speed...means SLOW PRINTS. i.e. we want to rampDown close to end so that
  // when we have finished ramping down, we only have a few mm left.
  if (dist_remaining + INT2MM(next_distance_remaining) > rampDistance(speed_step_count, step_increment, extruder_distance, speed)) {
      double custom_move = dist_remaining + INT2MM(next_distance_remaining) - rampDistance(speed_step_count, step_increment, extruder_distance, speed) - move_contingency;
      *output_stream << "; custom_move: " << custom_move << new_line;
      new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 0, e_d, custom_move, next_distance_remaining, finished);
  }


  
//JOE8

  // Get number of splits we can do in the remaining part of this move.
  // TODO THIS IS WRONG AT PRESENT AS WE MIGHT HAVE MOVED SOME DISTANCE ABOVE!!
  max_splits = maxSteps(step_direction, new_dist_remaining, speed_step_count, step_increment, extruder_distance, speed, m_speed_step);


  // If we aren't breaking this into pieces, because the piece is TOO small...then we just have the chance of ONE step here .. POTENTIALLY
  if (max_splits == 0) {
     *output_stream << "; Small move down: " << new_dist_remaining << new_line;
     // We only want to ramp down IF we have fully extruded one EXTRUDER full in the previous step.
     // We STILL may be able to ramp down ONE notch if the distance since last ramp is greater than the previous extruder_distance
     //
     // It is important to note...that if  we do not try to COMMENCE ramp down part-way through a small step.
     if (e_speed_step == m_speed_step) {
        // Commence new RAMP UP.... as we aren't alrady in a ramp up ... and it has been ample distance since last one.
        e_speed_step++;
     } else {   // We continue to extrude at given speed.
        // Continue to extrude...
     }
     new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 0, e_d, 0, next_distance_remaining, finished);


     return new_dist_remaining;   // No point continuing on, because we have finished this "run"
  }  



  // Get number of steps we can do in this case.
  extrude_splits = std::min(speed_step_count - m_speed_step, max_splits);
  *output_stream << "; RAMP DOWN - extrude_splits: " << extrude_splits << ", m_speed_step: " << m_speed_step << new_line;

  int initial_speed_step = m_speed_step;
  for (int step = m_speed_step+1; step <= (extrude_splits + initial_speed_step); step++) {
     e_speed_step = step;
   
// *output_stream << "; step = " << step << new_line;

     // TODO - NEED TO CATER FOR SCENARIO WHERE WE FINISh OFF TRAVEL BEFORE WE DO THE SPLITS.
     new_dist_remaining = new_dist_remaining - extrudeBit(extruder_distance, e_speed_step, m_speed_step, x, y, z, speed, feature, 0, e_d, 0, next_distance_remaining, finished);

     // We work out the speed fraction....so we can set reduced speed AND reduced E
//     double pfactor = (speed - m_speed_step * step_increment) / speed;

     // Do work - generate the line
     // TODO - Calculate the new extruder_distance..... since it is reduced based on pfactor
     // TODO - Calculate the new E value (ratio of extruder_distance to diff_length x e_delta)
     // TODO - Calculate the new speed (x by pfactor)
     // TODO - Generate new X,Y,Z points   (ratio of extruder_distance to diff_length x Each component)
     // TODO - Generate new G1 Code line
  }


//JOE9
  // IF we have finished ramping down, we make sure it is set that way.
  if (m_speed_step == speed_step_count) {
     step_direction = 2;   // So we don't do any more ramping down... This is not reset again until premove extrude is done.
  }
  return new_dist_remaining;
}



/*
 * e_speed_step - Speed step at which we extrude
 * m_speed_step - Speed step at which we move extruder
 *
 * We are wantint to keep in step with what is STILL falling out...m_speed_step
 * BUT... we want to speed up the extrusion of material.... hence e_speed_step
 *
 * Essentially, we need to extrude at an increased rate of p_factor / q_factor 
 *
 * extruder_distance - the distance the extruder moves in x,y at max speed
 * p_e_speed_step    - The speed at which the extrusion takes place...the step
 * p_m_speed_step    - The speed at which the extruder moves in x,y
 * x                 - Final x position
 * y                 - Final y position
 * z                 - Final z position
 * e                 - The final amount to extrude
 * speed             - The speed to go at
 * features          - just something we pass through
 * final_move        - Indicates if we try to finish up here...and get to x,y,z
 * e_d               - The density of e for give distance e/d
 * e_delta           - The amount we want to extrude over the WHOLE distance
 * next_distance_remaining - How much further we need to move extruder after this move to complete the path
 * finished          - Returns true or false, depending upon if we have finished the path.
 *
 *
 *
 *
*/

double GCodeExport::extrudeBit(double extruder_distance, int p_e_speed_step, int p_m_speed_step, int x, int y, int z, 
                               const Velocity &speed,const PrintFeatureType& feature, int final_move, double e_d, double cust_move,
                               int next_distance_remaining, bool *finished)
{
   double extruder_move, remaining_distance;   // Keeping track of extruder moves in X, Y plane
   double distance_moved;
   double e_change;   // Amount we actually extrude
   double factor;


   *finished = false;    // Default it

   // Distance the extruder moves  in x, y plane
   Point3 cp1 = currentPosition;
   remaining_distance = INT2MM(sqrt ((x - cp1.x) * (x - cp1.x) + (y - cp1.y) * (y - cp1.y)));
   double total_distance_remaining = INT2MM(next_distance_remaining) + remaining_distance;


   // Derive the speed factors for extrusion and xy move
   double mfactor = (speed - p_m_speed_step * step_increment) / speed;  // No change in speed from last time.



   // Work out what scenario we are dealing with to get distance.
   if (cust_move > 0 ) {
      extruder_move = cust_move;
      e_change = extruder_move * e_d;

   } else if (p_e_speed_step != p_m_speed_step)  { // We are ramping up or ramping down.
      // It is important to note that previous code ensures ramping up and ramping down is not right at the end...
      // Calculate how far we are moving the extruder in this move... So we are 
      extruder_move = std::min((getExtrudeDistance (extruder_distance, p_m_speed_step, speed) - travel_dist_since_step), remaining_distance);

      // Work out if this means we are at the end.
      if (getExtrudeDistance (extruder_distance, p_m_speed_step, speed) - travel_dist_since_step < remaining_distance) {
         *finished = false;
      } else {
         *finished = true;
      }


      // Update travel_dst_since_step
      travel_dist_since_step = travel_dist_since_step + extruder_move;

      // Calculate new delta E... that we can extrude over distance ....
      e_change = getExtrudeDistance (extruder_distance, p_e_speed_step, speed) * e_d * (extruder_move / getExtrudeDistance (extruder_distance, p_m_speed_step, speed));

// *output_stream << "; scenario 1 with e_change = " << e_change << ", extruder move: " << extruder_move << ", extruder_distance: " << extruder_distance << ", p_m_speed_step: " << p_m_speed_step << ", speed: " << speed << ", travel_dist_since_step: " << travel_dist_since_step << ", remaining_distance: " << remaining_distance << ", p_e_speed_step:"  << p_e_speed_step << ", final_move: " <<  final_move << new_line;
   } else if (total_distance_remaining > getExtrudeDistance (extruder_distance, p_m_speed_step, speed) && 
              INT2MM(next_distance_remaining) <= 0) {
      // WE ARE FINIShING OFF EXTRUDING - We WANT TO finish up exactly where the last E value should get us.
      // We want to ensure we have just enough powder left in outlet for last bit

      extruder_move = total_distance_remaining - getExtrudeDistance (extruder_distance, p_m_speed_step, speed);
      e_change = extruder_move * e_d;

// *output_stream << "; scenario 2" << new_line;
   } else if (total_distance_remaining > getExtrudeDistance (extruder_distance, p_m_speed_step, speed) && 
              INT2MM(next_distance_remaining) > 0 &&
              INT2MM(next_distance_remaining) < getExtrudeDistance (extruder_distance, p_m_speed_step, speed) ) {


      extruder_move = total_distance_remaining - getExtrudeDistance (extruder_distance, p_m_speed_step, speed);
      e_change = extruder_move * e_d;
// *output_stream << "; scenario 3" << new_line;
   } else if (total_distance_remaining < getExtrudeDistance (extruder_distance, p_m_speed_step, speed)) {
      // Not ramping up or ramping down, so we just move the WHOLE distance.
      extruder_move = remaining_distance;
      e_change = 0;    // No more extrusion...
      *finished = true;

// *output_stream << "; scenario 4" << new_line;
   } else {
      // All other scenarios....Not ramping up or ramping down, so we just move the WHOLE distance.
      extruder_move = remaining_distance;
      e_change = remaining_distance  * e_d;
      *finished = true;
// *output_stream << "; scenario 5" << new_line;
   }




   // Determine if this is TWO moves, or one move
   // ONE move if we going straight to the end...i.e. short move and we need to extrude at new speed ALL the way
   // TWO moves...if we are finishing up...part way along...and then we need to finish off at new speed.
   if (extruder_move < remaining_distance) {
        // By virtue of the fact it is less...this suggest e_speed_step is != m_speed_step
       
        // FIRST MOVE
        // Deduce timing and new position
        double x3 = cp1.x + (x - cp1.x) * extruder_move / remaining_distance;
        double y3 = cp1.y + (y - cp1.y) * extruder_move / remaining_distance;
        double z3 = cp1.z + (z - cp1.z) * extruder_move / remaining_distance;

// *output_stream << ";xb1"<<new_line;
   
        // Distance the extruder moves  in x, y plane
        distance_moved = INT2MM(sqrt ((x3 - cp1.x) * (x3 - cp1.x) + (y3 - cp1.y) * (y3 - cp1.y)));
      
        // Still need to ensure we get the right speed factor
        factor = totalSpeedFactor (cp1, x3, y3, e_change);


        *output_stream << "G1";
        writeFXYZE(Velocity(mfactor * Fxy * factor), x3, y3, z3, current_e_value + e_change, feature);


        m_speed_step = p_e_speed_step;   // Now we have them all at the SAME speed step.
        travel_dist_since_step = 0;      // So we must have finished

// I don't think we need this line. It screws up distance_moved if final_move == 1
//        remaining_distance = remaining_distance - distance_moved;


        // The final move never involves extrusion. There must be no more movement for this path 
        // after this.
        if (final_move == 1 && next_distance_remaining <= 0) {
           // SECOND MOVE
           mfactor = (speed - m_speed_step * step_increment) / speed;  // No change in speed from last time.
       
// *output_stream << ";xb2"<<new_line;
           e_change = 0;
           *output_stream << "G1";
           writeFXYZE(Velocity(mfactor * Fxy * factor), x, y, z, current_e_value + e_change, feature);

           distance_moved = remaining_distance;


           premove_extrude = 0;
           Fxy = 0;
           *output_stream << "; Finished up. next_distance_remaining: " << next_distance_remaining << new_line;
           *finished = true;

        }
   } else {

        // Just ONE MOVE
// *output_stream << ";xb3"<<new_line;
        *output_stream << "G1";
        factor = totalSpeedFactor (cp1, x, y, e_change);
        writeFXYZE(Velocity(mfactor * Fxy * factor), x, y, z, current_e_value + e_change, feature);

        distance_moved = remaining_distance;
        *finished = true;
   }


   return distance_moved;

}



int GCodeExport::maxSteps(int step_direction, double dist_remaining, int speed_step_count, int step_increment, double extruder_distance, const Velocity &speed, int speed_step)
{
   int num_steps = 0;
   double pfactor;
   double dist = 0;
   
   if (speed_step_count == 0) {  // IF we have no steps to do..... return 0
      return 0;
   }

   if (step_direction == 1) {  // Ramping UP
      if (speed_step > 0) {   // Only if we have more ramping up do we consider this...
         for (int i = speed_step; i >= 0; i--) {
             pfactor = (speed - i * step_increment) / speed;
             dist = dist + extruder_distance * pfactor;
             if (dist > dist_remaining) {
                break;
             }
             num_steps++;
         }
      }

   } else {
      // Going backawards
      if (speed_step < speed_step_count) {  // Only if we have more ramping down, do we consider this.
         for (int i = speed_step; i <= speed_step_count; i++) {
            pfactor = (speed - i * step_increment) / speed;
             dist = dist + extruder_distance * pfactor;
// *output_stream << "; Adding: " << extruder_distance * pfactor << ", TOTAL = " << dist << ", Comparing to :" << dist_remaining << new_line;
             if (dist > dist_remaining) {
                break;
             }
             num_steps++;
         }

      }
   }

 
   return num_steps;

}



double GCodeExport::rampDistance(int speed_step_count, int step_increment, double extruder_distance, const Velocity &speed)
{
   double ramp_distance = 0;
   double pfactor;

   for (int i = 0; i <= speed_step_count; i++) {
      pfactor = (speed - i * step_increment) / speed;
      ramp_distance = ramp_distance + extruder_distance * pfactor;
    }

   return ramp_distance;
}


double GCodeExport::effectiveSpeed(const Point3& cp1, const int x, const int y, const double e, const Velocity &speed)
{
   double effective_speed;

   double dx, dy, de;
 
   dx = INT2MM(x) - INT2MM(cp1.x);
   dy = INT2MM(y) - INT2MM(cp1.y);
   de = e;

   effective_speed = speed * sqrt((dx * dx + dy * dy) / (dx * dx + dy * dy + de * de));

   return effective_speed;
}


double GCodeExport::totalTime(const Point3& cp, const int x, const int y, const double e,  double speed)
{
   double total_time;

   double dx, dy, de;

   dx = INT2MM(x) - INT2MM(cp.x);
   dy = INT2MM(y) - INT2MM(cp.y);
   de = e;

   total_time = sqrt(dx * dx + dy * dy + de * de) / speed;

   return total_time;
}


double GCodeExport::totalSpeedFactor(const Point3& cp1, const int x, const int y, const double e)
{
   double total_speed_factor;

   double dx, dy, de;
 
   dx = INT2MM(x) - INT2MM(cp1.x);
   dy = INT2MM(y) - INT2MM(cp1.y);
   de = e;


   total_speed_factor = sqrt( (dx * dx + dy * dy + de * de) / (dx * dx + dy * dy) );

   return total_speed_factor;
}


/* 
 * This does the actual command writing
 *
 * Note: I expect that slowing down, speeding up will potentially cause issues with powder continuity...
 *       but because speeds are low (10mm/sec ... which might go to 20mm/sec), we are talking about VERY small 
 *       timeframes...which I'm hoping will result in seamless output..
 *
*/
void GCodeExport::writeFXYZEpart(const Velocity& speed, const int x, const int y, const int z, const double e, const PrintFeatureType& feature)
{

    if (currentSpeed != speed)
    {
        *output_stream << " F" << PrecisionedDouble{1, speed * 60};
        currentSpeed = speed;
    }

    Point gcode_pos = getGcodePos(x, y, current_extruder);
    total_bounding_box.include(Point3(gcode_pos.X, gcode_pos.Y, z));

    *output_stream << " X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y};
    if (z != currentPosition.z)
    {
        *output_stream << " Z" << MMtoStream{z};
    }
    if (e + current_e_offset != current_e_value)
    {
        const double output_e = (relative_extrusion)? e + current_e_offset - current_e_value : e + current_e_offset;
        *output_stream << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, output_e};
    }
    *output_stream << new_line;
    
    currentPosition = Point3(x, y, z);


    // Update values
    current_e_value_abs += (e - current_e_value); // TODO
    current_e_value = e;

    // log("writeFXYZE: current_e_value_abs: %f\n", current_e_value_abs);

    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(x), INT2MM(y), INT2MM(z), eToMm(e)), speed, feature);

}



void GCodeExport::writeFE(const Velocity& speed, const double e, const PrintFeatureType& feature)
{
    if (currentSpeed != speed)
    {
        *output_stream << " F" << PrecisionedDouble{1, speed * 60};
        currentSpeed = speed;
    }

    if (e + current_e_offset != current_e_value)
    {
        const double output_e = (relative_extrusion)? e + current_e_offset - current_e_value : e + current_e_offset;
        *output_stream << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, output_e};
    }
    *output_stream << new_line;

    // Update values
    current_e_value_abs += (e - current_e_value); // TODO
    current_e_value = e;


    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(e)), speed, feature);
}


void GCodeExport::writeUnretractionAndPrime()
{
    const double prime_volume = extruder_attr[current_extruder].prime_volume;
    const double prime_volume_e = mm3ToE(prime_volume);
    current_e_value += prime_volume_e;
    current_e_value_abs += prime_volume_e;
    if (extruder_attr[current_extruder].retraction_e_amount_current)
    {
        const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
        if (extruder_settings.get<bool>("machine_firmware_retract"))
        { // note that BFB is handled differently
            *output_stream << "G11" << new_line;
            //Assume default UM2 retraction settings.
            if (prime_volume != 0)
            {
                const double output_e = (relative_extrusion)? prime_volume_e : current_e_value;
                *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, output_e} << new_line;
                currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            }
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), 25.0, PrintFeatureType::MoveRetraction);
        }
        else
        {
            current_e_value += extruder_attr[current_extruder].retraction_e_amount_current;
            current_e_value_abs += extruder_attr[current_extruder].retraction_e_amount_current;
            const double output_e = (relative_extrusion)? extruder_attr[current_extruder].retraction_e_amount_current + prime_volume_e : current_e_value;
            *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, output_e} << new_line;
            currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
        }
    }
    else if (prime_volume != 0.0)
    {
        const double output_e = (relative_extrusion)? prime_volume_e : current_e_value;
        *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter;
        *output_stream << PrecisionedDouble{5, output_e} << new_line;
        currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::NoneType);
    }
    extruder_attr[current_extruder].prime_volume = 0.0;
    
    if (getCurrentExtrudedVolume() > 10000.0 && flavor != EGCodeFlavor::BFB && flavor != EGCodeFlavor::MAKERBOT) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
    {
        resetExtrusionValue();
    }
    if (extruder_attr[current_extruder].retraction_e_amount_current)
    {
        extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
    }
}

void GCodeExport::writeRetraction(const RetractionConfig& config, bool force, bool extruder_switch)
{
    ExtruderTrainAttributes& extr_attr = extruder_attr[current_extruder];

    if(flavor == EGCodeFlavor::BFB) //BitsFromBytes does automatic retraction.
    {
        if(extruder_switch)
        {
            if(!extr_attr.retraction_e_amount_current)
            {
                *output_stream << "M103" << new_line;
            }
            extr_attr.retraction_e_amount_current = 1.0; // 1.0 is a stub; BFB doesn't use the actual retracted amount; retraction is performed by firmware
        }
        return;
    }

    double old_retraction_e_amount = extr_attr.retraction_e_amount_current;
    double new_retraction_e_amount = mmToE(config.distance);
    double retraction_diff_e_amount = old_retraction_e_amount - new_retraction_e_amount;
    if(std::abs(retraction_diff_e_amount) < 0.000001)
    {
        return;
    }

    { // handle retraction limitation
        double current_extruded_volume = getCurrentExtrudedVolume();
        std::deque<double>& extruded_volume_at_previous_n_retractions = extr_attr.extruded_volume_at_previous_n_retractions;
        while (extruded_volume_at_previous_n_retractions.size() > config.retraction_count_max && !extruded_volume_at_previous_n_retractions.empty()) 
        {
            // extruder switch could have introduced data which falls outside the retraction window
            // also the retraction_count_max could have changed between the last retraction and this
            extruded_volume_at_previous_n_retractions.pop_back();
        }
        if(!force && config.retraction_count_max <= 0)
        {
            return;
        }
        if(!force && extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max
            && current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config.retraction_extrusion_window * extr_attr.filament_area) 
        {
            return;
        }
        extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
        if(extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max + 1) 
        {
            extruded_volume_at_previous_n_retractions.pop_back();
        }
    }

    const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if(extruder_settings.get<bool>("machine_firmware_retract"))
    {
        if(extruder_switch && extr_attr.retraction_e_amount_current) 
        {
            return; 
        }
        *output_stream << "G10";
        if(extruder_switch && flavor == EGCodeFlavor::REPETIER)
        {
            *output_stream << " S1";
        }
        *output_stream << new_line;
        //Assume default UM2 retraction settings.
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value + retraction_diff_e_amount)), 25, PrintFeatureType::MoveRetraction); // TODO: hardcoded values!
    }
    else
    {
        double speed = ((retraction_diff_e_amount < 0.0)? config.speed : extr_attr.last_retraction_prime_speed);
        current_e_value += retraction_diff_e_amount;
        current_e_value_abs += retraction_diff_e_amount;
        const double output_e = (relative_extrusion)? retraction_diff_e_amount : current_e_value;
        *output_stream << "G1 F" << PrecisionedDouble{1, speed * 60} << " " << extr_attr.extruderCharacter << PrecisionedDouble{5, output_e} << new_line;
        currentSpeed = speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
        extr_attr.last_retraction_prime_speed = config.primeSpeed;
    }

    extr_attr.retraction_e_amount_current = new_retraction_e_amount; // suppose that for UM2 the retraction amount in the firmware is equal to the provided amount
    extr_attr.prime_volume += config.prime_volume;

}

void GCodeExport::writeZhopStart(const coord_t hop_height, Velocity speed/*= 0*/)
{
    if (hop_height > 0)
    {
        if (speed == 0)
        {
            const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[current_extruder];
            speed = extruder.settings.get<Velocity>("speed_z_hop");
        }
        is_z_hopped = hop_height;
        currentSpeed = speed;
        *output_stream << "G1 F" << PrecisionedDouble{1, speed * 60} << " Z" << MMtoStream{current_layer_z + is_z_hopped} << new_line;
        total_bounding_box.includeZ(current_layer_z + is_z_hopped);
        assert(speed > 0.0 && "Z hop speed should be positive.");
    }
}

void GCodeExport::writeZhopEnd(Velocity speed/*= 0*/)
{
    if (is_z_hopped)
    {
        if (speed == 0)
        {
            const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[current_extruder];
            speed = extruder.settings.get<Velocity>("speed_z_hop");
        }
        is_z_hopped = 0;
        currentPosition.z = current_layer_z;
        currentSpeed = speed;
        *output_stream << "G1 F" << PrecisionedDouble{1, speed * 60} << " Z" << MMtoStream{current_layer_z} << new_line;
        assert(speed > 0.0 && "Z hop speed should be positive.");
    }
}

void GCodeExport::startExtruder(const size_t new_extruder)
{
    extruder_attr[new_extruder].is_used = true;
    if (new_extruder != current_extruder) // wouldn't be the case on the very first extruder start if it's extruder 0
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
        {
            *output_stream << "M135 T" << new_extruder << new_line;
        }
        else
        {
            *output_stream << "T" << new_extruder << new_line;
        }
    }

    current_extruder = new_extruder;

    assert(getCurrentExtrudedVolume() == 0.0 && "Just after an extruder switch we haven't extruded anything yet!");
    resetExtrusionValue(); // zero the E value on the new extruder, just to be sure

    const std::string start_code = Application::getInstance().current_slice->scene.extruders[new_extruder].settings.get<std::string>("machine_extruder_start_code");
    const double prime_amount = Application::getInstance().current_slice->scene.extruders[new_extruder].settings.get<double>("machine_extruder_prime_amount");
    const double prime_start_x = Application::getInstance().current_slice->scene.extruders[new_extruder].settings.get<double>("machine_extruder_prime_start_x");
    const double prime_start_y = Application::getInstance().current_slice->scene.extruders[new_extruder].settings.get<double>("machine_extruder_prime_start_y");
   

    if(!start_code.empty())
    {
        if (relative_extrusion)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
        }

        writeCode(start_code.c_str());

        primeExtruder(prime_amount, prime_start_x, prime_start_y);

        if (relative_extrusion)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }

    Application::getInstance().communication->setExtruderForSend(Application::getInstance().current_slice->scene.extruders[new_extruder]);
    Application::getInstance().communication->sendCurrentPosition(getPositionXY());

    //Change the Z position so it gets re-written again. We do not know if the switch code modified the Z position.
    currentPosition.z += 1;

    setExtruderFanNumber(new_extruder);
}


void GCodeExport::primeExtruder(double prime_amount, int prime_start_x, int prime_start_y)
{

    int y_end = prime_start_y - 20;

    // Move to the very right, to where we start the prime
    *output_stream << "G1 X10 F1500" << new_line;

    // Move to starting position
    *output_stream << "G1 X" << prime_start_x << " Y" << prime_start_y << " F1500" << new_line;

    // Prime a little
    current_e_value += prime_amount;
    current_e_value_abs += prime_amount;
    *output_stream << "G1 Y" << y_end << " E" << current_e_value << " F400" << new_line;

    // Move left a little
    // *output_stream << "G1 X12 F250" << new_line;

    // Prime a little more.
    current_e_value += prime_amount;
    current_e_value_abs += prime_amount;
    *output_stream << "G1 Y" << prime_start_y << " E" << current_e_value << " F400" << new_line;

    // Move left a little
    // *output_stream << "G1 X14 F250" << new_line;

    // Prime a little more.
    current_e_value += prime_amount;
    current_e_value_abs += prime_amount;
    *output_stream << "G1 Y" << y_end << " E" << current_e_value << " F400" << new_line;

    // Move left a little
    // *output_stream << "G1 X16 F250" << new_line;

    // Prime a little more.
    current_e_value += prime_amount;
    current_e_value_abs += prime_amount;
    *output_stream << "G1 Y" << prime_start_y << " E" << current_e_value << " F400" << new_line;


// The next two moves are to stop us hitting the container.
    // Move Z axis a little... so we can hop over things.
    *output_stream << "G1 Z60 F1500" << new_line;
    
    // Move to the CENTRE....so we are well and truely in the print zone.
    *output_stream << "G1 X100 Y100 F1500" << new_line;
    
    // Pause a little... to wait for all powder to drop out
    *output_stream << "G4 P500" << new_line;

}
 


void GCodeExport::finishExtruder()
{

    const Settings& old_extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;

    const std::string pre_end_code = old_extruder_settings.get<std::string>("machine_extruder_pre_end_code");

    const std::string end_code = old_extruder_settings.get<std::string>("machine_extruder_end_code");

    const double prime_start_x = old_extruder_settings.get<double>("machine_extruder_prime_start_x");

    const double prime_start_y = old_extruder_settings.get<double>("machine_extruder_prime_start_y");


    if(!end_code.empty() && !pre_end_code.empty())
    {
        if (relative_extrusion)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
        }

        // Move to dump location
        writeCode(pre_end_code.c_str());


        // New code to ensure we can disEngage motor from the Extruder Mechanism - METAL
        disEngageMotor(prime_start_x, prime_start_y);

        writeCode(end_code.c_str());

        if (relative_extrusion)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }

}


void GCodeExport::disEngageMotor(int x, int y)
{

   log("Doing Motor Dis-engage...\n");

   double angle_offset = 56;    // # of degrees that the Auger is ahead of stepper motor points
   // double e_per_revolution = M_PI * 11.0000; // Did 10 rotations for E340 So 1 rev is E34. (Close to M_PI * 11...but not quite)
   // double e_per_revolution = M_PI * 10.9000; // Did 10 rotations for E340 So 1 rev is E34. (Close to M_PI * 11...but not quite)
   // double e_per_revolution = 34.17335;   // Determined by trial and error.
   // double e_per_revolution = 34.5;   // Determined by trial and error.
   // double e_per_revolution = 34.41126;
   // double e_per_revolution = 34.4187484418598;
   // double e_per_revolution = 34.41817864129;
   // double e_per_revolution = 34.41126;
   double e_per_revolution = 34.40916795;
   double e_half_turn = e_per_revolution / 2;     // Helpers
   double e_quarter_turn = e_per_revolution / 4;  // Helpers
   double e_offset = e_per_revolution * (angle_offset / 360);

   // Work out how far to get precisely back to starting position.
   double e_move = e_per_revolution * (1 - ((current_e_value_abs / e_per_revolution) - int(current_e_value_abs / e_per_revolution)));

   // With the above, it will always put this back to the original position ....which might be more than we have to...
   if (e_move > e_half_turn) {
      e_move = e_move - e_half_turn;
   }

   // Work out how far to move back to position where the AUGER contact points will be aligned along the X-Axis.
   e_move = e_move + e_quarter_turn - e_offset;


   // Move to position...
   *output_stream << "; Moving to position before zeroing extruder" << new_line;
   *output_stream << "G1 X" << x << " Y" << y << " F1500" << new_line;


   double current_angle = (current_e_value_abs / e_per_revolution - int(current_e_value_abs / e_per_revolution)) * 360;
   log("current_e_value_abs: %f\n", current_e_value_abs);
   log("Current Angle: %f\n", current_angle);

   // log("FIRST MOVE: %f\n", e_move);
   current_e_value += e_move;  // DO we REALLY Need to record this?
   current_e_value_abs += e_move;

   *output_stream << "G1 E" << current_e_value << " F500" << new_line;
   *output_stream << "; Acurrent_e_value_abs = " << current_e_value_abs << new_line;


   current_angle = (current_e_value_abs / e_per_revolution - int(current_e_value_abs / e_per_revolution)) * 360;
   log("AFTER FIRST_MOVE: %f\n", current_angle);
   
   // SECOND MOVE
   e_move = -(e_quarter_turn - e_offset);

   

   // writeCode(e_move_str);
   // log("SECOND MOVE: %f\n", e_move);
   current_e_value += e_move;   // DO we REALLY Need to record this?
   current_e_value_abs += e_move;

   *output_stream << "G1 E" << current_e_value << " F500" << new_line;
   *output_stream << "; Zcurrent_e_value_abs = " << current_e_value_abs << new_line;

   current_angle = (current_e_value_abs / e_per_revolution - int(current_e_value_abs / e_per_revolution)) * 360;
   log("AFTER SECOND_MOVE: %f\n", current_angle);
}


void GCodeExport::switchExtruder(size_t new_extruder, const RetractionConfig& retraction_config_old_extruder, coord_t perform_z_hop /*= 0*/)
{
    if (current_extruder == new_extruder)
    {
        return;
    }

    const Settings& old_extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if(old_extruder_settings.get<bool>("retraction_enable"))
    {
        constexpr bool force = true;
        constexpr bool extruder_switch = true;
        writeRetraction(retraction_config_old_extruder, force, extruder_switch);
    }

    if (perform_z_hop > 0)
    {
        writeZhopStart(perform_z_hop);
    }

    resetExtrusionValue(); // zero the E value on the old extruder, so that the current_e_value is registered on the old extruder

    const std::string pre_end_code = old_extruder_settings.get<std::string>("machine_extruder_pre_end_code");

    const std::string end_code = old_extruder_settings.get<std::string>("machine_extruder_end_code");

    const double prime_start_x = old_extruder_settings.get<double>("machine_extruder_prime_start_x");

    const double prime_start_y = old_extruder_settings.get<double>("machine_extruder_prime_start_y");

    if(!end_code.empty() && !pre_end_code.empty())
    {
        if (relative_extrusion)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
        }

        // Move to dump location
        writeCode(pre_end_code.c_str());

        // New code to ensure we can disEngage motor from the Extruder Mechanism - METAL
        disEngageMotor(prime_start_x, prime_start_y);

        writeCode(end_code.c_str());

        if (relative_extrusion)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }

    startExtruder(new_extruder);
}




void GCodeExport::writeCode(const char* str)
{
    *output_stream << str << new_line;
}

void GCodeExport::writePrimeTrain(const Velocity& travel_speed)
{
    if (extruder_attr[current_extruder].is_primed)
    { // extruder is already primed once!
        return;
    }
    const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if (extruder_settings.get<bool>("prime_blob_enable"))
    { // only move to prime position if we do a blob/poop
        // ideally the prime position would be respected whether we do a blob or not,
        // but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
        // which is needed to automatically ignore the prime position for the printer when blob is disabled
        Point3 prime_pos(extruder_settings.get<coord_t>("extruder_prime_pos_x"), extruder_settings.get<coord_t>("extruder_prime_pos_y"), extruder_settings.get<coord_t>("extruder_prime_pos_z"));
        if (!extruder_settings.get<bool>("extruder_prime_pos_abs"))
        {
            // currentPosition.z can be already z hopped
            prime_pos += Point3(currentPosition.x, currentPosition.y, current_layer_z);
        }
        writeTravel(prime_pos, travel_speed);
    }

    if (flavor == EGCodeFlavor::GRIFFIN)
    {
        bool should_correct_z = false;
        
        std::string command = "G280";
        if (!extruder_settings.get<bool>("prime_blob_enable"))
        {
            command += " S1";  // use S1 to disable prime blob
            should_correct_z = true;
        }
        *output_stream << command << new_line;

        // There was an issue with the S1 strategy parameter, where it would only change the material-position,
        //   as opposed to 'be a prime-blob maneuvre without actually printing the prime blob', as we assumed here.
        // After a chat, the firmware-team decided to change the S1 strategy behaviour,
        //   but since people don't update their firmware at each opportunity, it was decided to fix it here as well.
        if (should_correct_z)
        {
            // Can't output via 'writeTravel', since if this is needed, the value saved for 'current height' will not be correct.
            // For similar reasons, this isn't written to the front-end via command-socket.
            *output_stream << "G0 Z" << MMtoStream{getPositionZ()} << new_line;
        }
    }
    else
    {
        // there is no prime gcode for other firmware versions...
    }

    extruder_attr[current_extruder].is_primed = true;
}

void GCodeExport::setExtruderFanNumber(int extruder)
{
    if (extruder_attr[extruder].fan_number != fan_number)
    {
        fan_number = extruder_attr[extruder].fan_number;
        current_fan_speed = -1; // ensure fan speed gcode gets output for this fan
    }
}

void GCodeExport::writeFanCommand(double speed)
{
    if (std::abs(current_fan_speed - speed) < 0.1)
    {
        return;
    }
    if(flavor == EGCodeFlavor::MAKERBOT)
    {
        if(speed >= 50)
        {
            *output_stream << "M126 T0" << new_line; //Makerbot cannot PWM the fan speed...
        }
        else
        {
            *output_stream << "M127 T0" << new_line;
        }
    }
    else if (speed > 0)
    {
        *output_stream << "M106 S" << PrecisionedDouble{1, speed * 255 / 100};
        if (fan_number)
        {
            *output_stream << " P" << fan_number;
        }
        *output_stream << new_line;
    }
    else
    {
        *output_stream << "M107";
        if (fan_number)
        {
            *output_stream << " P" << fan_number;
        }
        *output_stream << new_line;
    }

    current_fan_speed = speed;
}

void GCodeExport::writeTemperatureCommand(const size_t extruder, const Temperature& temperature, const bool wait)
{
    const ExtruderTrain& extruder_train = Application::getInstance().current_slice->scene.extruders[extruder];

    if (!extruder_train.settings.get<bool>("machine_nozzle_temp_enabled"))
    {
        return;
    }

    if (extruder_train.settings.get<bool>("machine_extruders_share_heater"))
    {
        // extruders share a single heater
        if (extruder != current_extruder)
        {
            // ignore all changes to the non-current extruder
            return;
        }

        // sync all extruders with the change to the current extruder
        const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();

        for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            if (extruder_nr != extruder)
            {
                // only reset the other extruders' waited_for_temperature state when the new temperature
                // is greater than the old temperature
                if (wait || temperature > extruder_attr[extruder_nr].currentTemperature)
                {
                    extruder_attr[extruder_nr].waited_for_temperature = wait;
                }
                extruder_attr[extruder_nr].currentTemperature = temperature;
            }
        }
    }

    if ((!wait || extruder_attr[extruder].waited_for_temperature) && extruder_attr[extruder].currentTemperature == temperature)
    {
        return;
    }

    if (wait && flavor != EGCodeFlavor::MAKERBOT)
    {
        if(flavor == EGCodeFlavor::MARLIN)
        {
            *output_stream << "M105" << new_line; // get temperatures from the last update, the M109 will not let get the target temperature
        }
        *output_stream << "M109";
        extruder_attr[extruder].waited_for_temperature = true;
    }
    else
    {
        *output_stream << "M104";
        extruder_attr[extruder].waited_for_temperature = false;
    }
    if (extruder != current_extruder)
    {
        *output_stream << " T" << extruder;
    }
#ifdef ASSERT_INSANE_OUTPUT
    assert(temperature >= 0);
#endif // ASSERT_INSANE_OUTPUT
    *output_stream << " S" << PrecisionedDouble{1, temperature} << new_line;
    if (extruder != current_extruder && always_write_active_tool)
    {
        //Some firmwares (ie Smoothieware) change tools every time a "T" command is read - even on a M104 line, so we need to switch back to the active tool.
        *output_stream << "T" << current_extruder << new_line;
    }
    if (wait && flavor == EGCodeFlavor::MAKERBOT)
    {
        //Makerbot doesn't use M109 for heat-and-wait. Instead, use M104 and then wait using M116.
        *output_stream << "M116" << new_line;
    }
    extruder_attr[extruder].currentTemperature = temperature;
}

void GCodeExport::writeBedTemperatureCommand(const Temperature& temperature, const bool wait)
{
    if (flavor == EGCodeFlavor::ULTIGCODE)
    { // The UM2 family doesn't support temperature commands (they are fixed in the firmware)
        return;
    }

    if (wait)
    {
        if(flavor == EGCodeFlavor::MARLIN)
        {
            *output_stream << "M140 S"; // set the temperature, it will be used as target temperature from M105
            *output_stream << PrecisionedDouble{1, temperature} << new_line;
            *output_stream << "M105" << new_line;
        }

        *output_stream << "M190 S";
    }
    else
        *output_stream << "M140 S";
    *output_stream << PrecisionedDouble{1, temperature} << new_line;
}

void GCodeExport::writeBuildVolumeTemperatureCommand(const Temperature& temperature, const bool wait)
{
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::GRIFFIN)
    {
        //Ultimaker printers don't support build volume temperature commands.
        return;
    }
    if (wait)
    {
        *output_stream << "M191 S";
    }
    else
    {
        *output_stream << "M141 S";
    }
    *output_stream << PrecisionedDouble{1, temperature} << new_line;
}

void GCodeExport::writePrintAcceleration(const Acceleration& acceleration)
{
    switch (getFlavor())
    {
        case EGCodeFlavor::REPETIER:
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M201 X" << PrecisionedDouble{0, acceleration} << " Y" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::REPRAP:
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M204 P" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        default:
            // MARLIN, etc. only have one acceleration for both print and travel
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M204 S" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
    }
    current_print_acceleration = acceleration;
    estimateCalculator.setAcceleration(acceleration);
}

void GCodeExport::writeTravelAcceleration(const Acceleration& acceleration)
{
    switch (getFlavor())
    {
        case EGCodeFlavor::REPETIER:
            if (current_travel_acceleration != acceleration)
            {
                *output_stream << "M202 X" << PrecisionedDouble{0, acceleration} << " Y" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::REPRAP:
            if (current_travel_acceleration != acceleration)
            {
                *output_stream << "M204 T" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        default:
            // MARLIN, etc. only have one acceleration for both print and travel
            writePrintAcceleration(acceleration);
            break;
    }
    current_travel_acceleration = acceleration;
    estimateCalculator.setAcceleration(acceleration);
}

void GCodeExport::writeJerk(const Velocity& jerk)
{
    if (current_jerk != jerk)
    {
        switch (getFlavor())
        {
            case EGCodeFlavor::REPETIER:
                *output_stream << "M207 X" << PrecisionedDouble{2, jerk} << new_line;
                break;
            case EGCodeFlavor::REPRAP:
                *output_stream << "M566 X" << PrecisionedDouble{2, jerk * 60} << " Y" << PrecisionedDouble{2, jerk * 60} << new_line;
                break;
            default:
                *output_stream << "M205 X" << PrecisionedDouble{2, jerk} << " Y" << PrecisionedDouble{2, jerk} << new_line;
                break;
        }
        current_jerk = jerk;
        estimateCalculator.setMaxXyJerk(jerk);
    }
}

void GCodeExport::finalize(const char* endCode)
{
    writeFanCommand(0);
    writeCode(endCode);
    int64_t print_time = getSumTotalPrintTimes();
    int mat_0 = getTotalFilamentUsed(0);
    log("Print time (s): %d\n", print_time);
    log("Print time (hr|min|s): %dh %dm %ds\n", int(print_time / 60 / 60), int((print_time / 60) % 60), int(print_time % 60));
    log("Filament (mm^3): %d\n", mat_0);
    for(int n=1; n<MAX_EXTRUDERS; n++)
        if (getTotalFilamentUsed(n) > 0)
            log("Filament%d: %d\n", n + 1, int(getTotalFilamentUsed(n)));
    output_stream->flush();
}

double GCodeExport::getExtrudedVolumeAfterLastWipe(size_t extruder)
{
    return eToMm3(extruder_attr[extruder].last_e_value_after_wipe, extruder);
}

void GCodeExport::ResetLastEValueAfterWipe(size_t extruder)
{
    extruder_attr[extruder].last_e_value_after_wipe = 0;
}

void GCodeExport::insertWipeScript(const WipeScriptConfig& wipe_config)
{
    const Point3 prev_position = currentPosition;
    writeComment("WIPE_SCRIPT_BEGIN");

    if (wipe_config.retraction_enable)
    {
        writeRetraction(wipe_config.retraction_config);
    }

    if (wipe_config.hop_enable)
    {
        writeZhopStart(wipe_config.hop_amount, wipe_config.hop_speed);
    }

    writeTravel(Point(wipe_config.brush_pos_x, currentPosition.y), wipe_config.move_speed);
    for (size_t i = 0; i < wipe_config.repeat_count; ++i)
    {
        coord_t x = currentPosition.x + (i % 2 ? -wipe_config.move_distance : wipe_config.move_distance);
        writeTravel(Point(x, currentPosition.y), wipe_config.move_speed);
    }

    writeTravel(prev_position, wipe_config.move_speed);

    if (wipe_config.hop_enable)
    {
        writeZhopEnd(wipe_config.hop_speed);
    }

    if (wipe_config.retraction_enable)
    {
        writeUnretractionAndPrime();
    }

    if (wipe_config.pause > 0)
    {
        writeDelay(wipe_config.pause);
    }

    writeComment("WIPE_SCRIPT_END");
}

}//namespace cura

