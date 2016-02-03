//
// Created by arssivka on 1/18/16.
//

#include <rd/control/Kinematics.h>

#include <bhuman/tools/InverseKinematic.h>
#include <bhuman/tools/ForwardKinematic.h>

using namespace AL;
using namespace std;
using namespace boost;
using namespace rd;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int chains_size[] = {Joints::ARM_JOINTS_COUNT,
                           Joints::ARM_JOINTS_COUNT,
                           Joints::LEG_JOINTS_COUNT,
                           Joints::LEG_JOINTS_COUNT};

const int first_servo[] = {Joints::LEFT_ARM_FIRST_JOINT,
                           Joints::RIGHT_ARM_FIRST_JOINT,
                           Joints::LEFT_LEG_FIRST_JOINT,
                           Joints::RIGHT_LEG_FIRST_JOINT};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Kinematics::Kinematics(shared_ptr<Clock> clock, shared_ptr<Joints> joints, shared_ptr<property_tree::ptree> config)
        : keys(KEYS_COUNT), clock(clock), joints(joints) {
    this->keys[LEFT_ARM_X] = std::string("LEFT_ARM_X");
    this->keys[LEFT_ARM_Y] = std::string("LEFT_ARM_Y");
    this->keys[LEFT_ARM_Z] = std::string("LEFT_ARM_Z");
    this->keys[RIGHT_ARM_X] = std::string("RIGHT_ARM_X");
    this->keys[RIGHT_ARM_Y] = std::string("RIGHT_ARM_Y");
    this->keys[RIGHT_ARM_Z] = std::string("RIGHT_ARM_Z");
    this->keys[LEFT_LEG_X] = std::string("LEFT_LEG_X");
    this->keys[LEFT_LEG_Y] = std::string("LEFT_LEG_Y");
    this->keys[LEFT_LEG_Z] = std::string("LEFT_LEG_Z");
    this->keys[LEFT_LEG_ROLL] = std::string("LEFT_LEG_ROLL");
    this->keys[LEFT_LEG_PITCH] = std::string("LEFT_LEG_PITCH");
    this->keys[LEFT_LEG_YAW] = std::string("LEFT_LEG_YAW");
    this->keys[RIGHT_LEG_X] = std::string("RIGHT_LEG_X");
    this->keys[RIGHT_LEG_Y] = std::string("RIGHT_LEG_Y");
    this->keys[RIGHT_LEG_Z] = std::string("RIGHT_LEG_Z");
    this->keys[RIGHT_LEG_ROLL] = std::string("RIGHT_LEG_ROLL");
    this->keys[RIGHT_LEG_PITCH] = std::string("RIGHT_LEG_PITCH");
    this->keys[RIGHT_LEG_YAW] = std::string("RIGHT_LEG_YAW");

    for (int i = 0; i < KEYS_COUNT; ++i)
        this->keys_map.insert(std::make_pair(this->keys[i], i));

    boost::property_tree::ptree properties, joint;
    properties = config->get_child("BHuman").get_child("JointCalibration");

    JointCalibration::JointInfo* info;
    for (int i = 0; i < Joints::JOINTS_COUNT; ++i) {
        // TODO This parameters should be in config file
        //if(i == Joints::L_HAND || i == Joints::R_HAND) continue;
        info = &this->joint_calibration.joints[i];
        joint = properties.get_child(joints->getKeys()[i]);
        info->offset = joint.get<float>("Offset");
        info->sign = joint.get<short>("Sign", 1);
        info->maxAngle = joint.get<float>("MaxAngle");
        info->minAngle = joint.get<float>("MinAngle");
    }
    properties = config->get_child("BHuman").get_child("RobotDimensions");
    this->robot_dimensions.lengthBetweenLegs = properties.get<float>("LengthBetweenLegs");
    this->robot_dimensions.upperLegLength = properties.get<float>("UpperLegLength");
    this->robot_dimensions.lowerLegLength = properties.get<float>("LowerLegLength");
    this->robot_dimensions.heightLeg5Joint = properties.get<float>("HeightLeg5Joint");
    this->robot_dimensions.zLegJoint1ToHeadPan = properties.get<float>("ZLegJoint1ToHeadPan");
    this->robot_dimensions.xHeadTiltToCamera = properties.get<float>("XHeadTiltToCamera");
    this->robot_dimensions.zHeadTiltToCamera = properties.get<float>("ZHeadTiltToCamera");
    this->robot_dimensions.headTiltToCameraTilt = properties.get<float>("HeadTiltToCameraTilt");
    this->robot_dimensions.xHeadTiltToUpperCamera = properties.get<float>("XHeadTiltToUpperCamera");
    this->robot_dimensions.zHeadTiltToUpperCamera = properties.get<float>("ZHeadTiltToUpperCamera");
    this->robot_dimensions.headTiltToUpperCameraTilt = properties.get<float>("HeadTiltToUpperCameraTilt");
    this->robot_dimensions.armOffset.x = properties.get_child("ArmOffset").get<float>("x");
    this->robot_dimensions.armOffset.y = properties.get_child("ArmOffset").get<float>("y");
    this->robot_dimensions.armOffset.z = properties.get_child("ArmOffset").get<float>("z");
    this->robot_dimensions.yElbowShoulder = properties.get<float>("YElbowShoulder");
    this->robot_dimensions.upperArmLength = properties.get<float>("UpperArmLength");
    this->robot_dimensions.lowerArmLength = properties.get<float>("LowerArmLength");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string>& Kinematics::getKeys() const {
    return this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Kinematics::lookAt(double x, double y, double z, bool top_camera) {
    std::vector<double> request_data(2);
    std::vector<int> request_keys(2);
    request_keys[0] = Joints::HEAD_YAW;
    request_keys[1] = Joints::HEAD_PITCH;
    Vector3<> position(x, y, z);
    InverseKinematic::calcHeadJoints(position, pi / 2, this->robot_dimensions, top_camera, request_data,
                                     this->camera_calibration);
    this->joints->setPositions(request_keys, request_data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Kinematics::setPosition(const vector<string>& keys, const vector<double>& values) {
    std::vector<int> k(keys.size());
    try {
        for (int i = 0; i < keys.size(); ++i)
            k[i] = this->keys_map.find(keys[i])->second;
    } catch (std::out_of_range& e) {
        return;
    }
    this->setPosition(k, values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Kinematics::setPosition(const vector<int>& keys, const vector<double>& values) {
    // Null checking
    if (keys.empty() || keys.size() != values.size())
        return;
    // Calculate mask
    int mask = 0;
    for (int i = 0; i < keys.size(); ++i)
        mask |= 1 << keys[i];
    // Check mask
    Pose3D limbs[MassCalibration::numOfLimbs];
    int required_limbs[LIMBS_COUNT];
    required_limbs[LEFT_ARM] = mask & LEFT_ARM_MASK;
    required_limbs[RIGHT_ARM] = mask & RIGHT_ARM_MASK;
    required_limbs[LEFT_LEG] = mask & LEFT_LEG_MASK;
    required_limbs[RIGHT_LEG] = mask & RIGHT_LEG_MASK;
    // Get joint data
    shared_ptr<SensorData<double> > data = this->joints->getPositions();
    // Calculate forward kinematics
    int request_size = 0;
    if (required_limbs[LEFT_ARM]) {
        if (required_limbs[LEFT_ARM] != LEFT_ARM_MASK)
            ForwardKinematic::calculateArmChain(true, data->data, this->robot_dimensions, this->mass_calibration,
                                                limbs);
        request_size += Joints::ARM_JOINTS_COUNT;
    }
    if (required_limbs[RIGHT_ARM]) {
        if (required_limbs[RIGHT_ARM] != RIGHT_ARM_MASK)
            ForwardKinematic::calculateArmChain(false, data->data, this->robot_dimensions, this->mass_calibration,
                                                limbs);
        request_size += Joints::ARM_JOINTS_COUNT;
    }
    if (required_limbs[LEFT_LEG] || required_limbs[RIGHT_LEG]) {
        if (required_limbs[LEFT_LEG] != LEFT_LEG_MASK)
            ForwardKinematic::calculateLegChain(true, data->data, this->robot_dimensions, this->mass_calibration,
                                                limbs);
        if (required_limbs[RIGHT_LEG] != RIGHT_LEG_MASK)
            ForwardKinematic::calculateLegChain(false, data->data, this->robot_dimensions, this->mass_calibration,
                                                limbs);
        request_size += 2 * Joints::LEG_JOINTS_COUNT;
    }

    cout << "Forward Ok" << endl;

    // Change variables
    for (int i = 0; i < keys.size(); ++i) {
        switch (keys[i]) {
            case LEFT_ARM_X:
                limbs[MassCalibration::foreArmLeft].translation.x = values[i];
                break;
            case LEFT_ARM_Y:
                limbs[MassCalibration::foreArmLeft].translation.y = values[i];
                break;
            case LEFT_ARM_Z:
                limbs[MassCalibration::foreArmLeft].translation.z = values[i];
                break;
            case RIGHT_ARM_X:
                limbs[MassCalibration::foreArmRight].translation.x = values[i];
                break;
            case RIGHT_ARM_Y:
                limbs[MassCalibration::foreArmRight].translation.y = values[i];
                break;
            case RIGHT_ARM_Z:
                limbs[MassCalibration::foreArmRight].translation.z = values[i];
                break;
            case LEFT_LEG_X:
                limbs[MassCalibration::ankleLeft].translation.x = values[i];
                break;
            case LEFT_LEG_Y:
                limbs[MassCalibration::ankleLeft].translation.y = values[i];
                break;
            case LEFT_LEG_Z:
                limbs[MassCalibration::ankleLeft].translation.z = values[i];
                break;
            case LEFT_LEG_ROLL:
                limbs[MassCalibration::ankleLeft].rotateX(values[i]);
                break;
            case LEFT_LEG_PITCH:
                limbs[MassCalibration::ankleLeft].rotateY(values[i]);
                break;
            case LEFT_LEG_YAW:
                limbs[MassCalibration::ankleLeft].rotateZ(values[i]);
                break;
            case RIGHT_LEG_X:
                limbs[MassCalibration::ankleRight].translation.x = values[i];
                break;
            case RIGHT_LEG_Y:
                limbs[MassCalibration::ankleRight].translation.y = values[i];
                break;
            case RIGHT_LEG_Z:
                limbs[MassCalibration::ankleRight].translation.z = values[i];
                break;
            case RIGHT_LEG_ROLL:
                limbs[MassCalibration::ankleRight].rotateX(values[i]);
                break;
            case RIGHT_LEG_PITCH:
                limbs[MassCalibration::ankleRight].rotateY(values[i]);
                break;
            case RIGHT_LEG_YAW:
                limbs[MassCalibration::ankleRight].rotateZ(values[i]);
                break;
            default:
                break;
        }
    }

    cout << "Positions Ok" << endl;

    // Prepare request
    std::vector<int> request_keys(request_size);
    std::vector<double> request_values(request_size);
    cout << "Request size" << request_size << endl;

    int index = 0;
    if (required_limbs[LEFT_ARM] || required_limbs[RIGHT_ARM])
        InverseKinematic::calcArmJoints(limbs[MassCalibration::foreArmLeft], limbs[MassCalibration::foreArmRight],
                                        data->data, this->robot_dimensions, this->joint_calibration);
    cout << "Inverce Arms ok" << endl;
    if (required_limbs[LEFT_ARM]) {
        for (int i = 0; i < Joints::ARM_JOINTS_COUNT; ++i) {
            int servo = first_servo[LEFT_ARM] + i;
            request_keys[index + i] = servo;
            request_values[index + i] = data->data[servo];
            cout << servo << endl;
        }
        index += Joints::ARM_JOINTS_COUNT;
    }
    cout << "Insert left arm ok" << endl;
    if (required_limbs[RIGHT_ARM]) {
        for (int i = 0; i < Joints::ARM_JOINTS_COUNT; ++i) {
            int servo = first_servo[RIGHT_ARM] + i;
            request_keys[index + i] = servo;
            request_values[index + i] = data->data[servo];
            cout << servo << endl;
        }
        index += Joints::ARM_JOINTS_COUNT;
    }
    cout << "Insert right arm ok" << endl;
    if (required_limbs[LEFT_LEG] || required_limbs[RIGHT_LEG]) {
        InverseKinematic::calcLegJoints(limbs[MassCalibration::ankleLeft], data->data, true, this->robot_dimensions);
        InverseKinematic::calcLegJoints(limbs[MassCalibration::ankleRight], data->data, false, this->robot_dimensions);
        cout << "Inverce legs ok" << endl;
        for (int i = 0; i < Joints::LEG_JOINTS_COUNT; ++i) {
            int servo = first_servo[LEFT_LEG] + i;
            request_keys[index + i] = servo;
            request_values[index + i] = data->data[servo];
            cout << servo << endl;
        }
        cout << "Insert left leg ok" << endl;
        index += Joints::ARM_JOINTS_COUNT;
        for (int i = 0; i < Joints::LEG_JOINTS_COUNT; ++i) {
            int servo = first_servo[RIGHT_LEG] + i;
            request_keys[index + i] = servo;
            request_values[index + i] = data->data[servo];
            cout << servo << endl;
        }
        cout << "Insert right leg ok" << endl;
    }
    this->joints->setPositions(request_keys, request_values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > Kinematics::getPosition(const vector<string>& keys) {
    std::vector<int> k(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        const map<string, int>::iterator& found = this->keys_map.find(keys[i]);
        if (found == this->keys_map.end())
            return make_shared<SensorData<double> >(0, clock->getTime(0));
        k[i] = found->second;
    }
    return this->getPosition(k);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > Kinematics::getPosition(const vector<int>& keys) {
    // Null checking
    if (keys.empty())
        return make_shared<SensorData<double> >(0, this->clock->getTime(0));
    // Calculate mask
    int mask = 0;
    for (int i = 0; i < keys.size(); ++i)
        mask |= 1 << keys[i];
    // Check mask
    Pose3D limbs[MassCalibration::numOfLimbs];
    int required_limbs[LIMBS_COUNT];
    required_limbs[LEFT_ARM] = mask & LEFT_ARM_MASK;
    required_limbs[RIGHT_ARM] = mask & RIGHT_ARM_MASK;
    required_limbs[LEFT_LEG] = mask & LEFT_LEG_MASK;
    required_limbs[RIGHT_LEG] = mask & RIGHT_LEG_MASK;
    // Get joint data
    shared_ptr<SensorData<double> > data = this->joints->getPositions();
    // Calculate forward kinematics
    if (required_limbs[LEFT_ARM])
        ForwardKinematic::calculateArmChain(true, data->data, this->robot_dimensions, this->mass_calibration, limbs);
    if (required_limbs[RIGHT_ARM])
        ForwardKinematic::calculateArmChain(false, data->data, this->robot_dimensions, this->mass_calibration, limbs);
    if (required_limbs[LEFT_LEG])
        ForwardKinematic::calculateLegChain(true, data->data, this->robot_dimensions, this->mass_calibration, limbs);
    if (required_limbs[RIGHT_LEG])
        ForwardKinematic::calculateLegChain(false, data->data, this->robot_dimensions, this->mass_calibration, limbs);
    shared_ptr<SensorData<double> > result = make_shared<SensorData<double> >(keys.size(), clock->getTime(0));
    for (int i = 0; i < keys.size(); ++i) {
        switch (keys[i]) {
            case LEFT_ARM_X:
                result->data[i] = limbs[MassCalibration::foreArmLeft].translation.x;
                break;
            case LEFT_ARM_Y:
                result->data[i] = limbs[MassCalibration::foreArmLeft].translation.y;
                break;
            case LEFT_ARM_Z:
                result->data[i] = limbs[MassCalibration::foreArmLeft].translation.z;
                break;
            case RIGHT_ARM_X:
                result->data[i] = limbs[MassCalibration::foreArmRight].translation.x;
                break;
            case RIGHT_ARM_Y:
                result->data[i] = limbs[MassCalibration::foreArmRight].translation.y;
                break;
            case RIGHT_ARM_Z:
                result->data[i] = limbs[MassCalibration::foreArmRight].translation.z;
                break;
            case LEFT_LEG_X:
                result->data[i] = limbs[MassCalibration::ankleLeft].translation.x;
                break;
            case LEFT_LEG_Y:
                result->data[i] = limbs[MassCalibration::ankleLeft].translation.y;
                break;
            case LEFT_LEG_Z:
                result->data[i] = limbs[MassCalibration::ankleLeft].translation.z;
                break;
            case LEFT_LEG_ROLL:
                result->data[i] = limbs[MassCalibration::ankleLeft].rotation.getXAngle();
                break;
            case LEFT_LEG_PITCH:
                result->data[i] = limbs[MassCalibration::ankleLeft].rotation.getYAngle();
                break;
            case LEFT_LEG_YAW:
                result->data[i] = limbs[MassCalibration::ankleLeft].rotation.getZAngle();
                break;
            case RIGHT_LEG_X:
                result->data[i] = limbs[MassCalibration::ankleRight].translation.x;
                break;
            case RIGHT_LEG_Y:
                result->data[i] = limbs[MassCalibration::ankleRight].translation.y;
                break;
            case RIGHT_LEG_Z:
                result->data[i] = limbs[MassCalibration::ankleRight].translation.z;
                break;
            case RIGHT_LEG_ROLL:
                result->data[i] = limbs[MassCalibration::ankleRight].rotation.getXAngle();
                break;
            case RIGHT_LEG_PITCH:
                result->data[i] = limbs[MassCalibration::ankleRight].rotation.getYAngle();
                break;
            case RIGHT_LEG_YAW:
                result->data[i] = limbs[MassCalibration::ankleRight].rotation.getZAngle();
                break;
            default:
                break;
        }
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > Kinematics::getPosition() {
    return this->getPosition(this->keys);
}