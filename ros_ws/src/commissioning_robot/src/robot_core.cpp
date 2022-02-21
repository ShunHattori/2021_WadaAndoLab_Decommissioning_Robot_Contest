#include <utils.hpp>

ds4_driver::Status ds4_status;
ds4_driver::Feedback ds4_feedback;
geometry_msgs::Twist robot_velocity;
geometry_msgs::Twist drive_inverse_kinematics;
commissioning_robot::ControlState C_pose_controller;
commissioning_robot::ControlState C_drive_wheel_controller;
commissioning_robot::ControlState C_main_lift_controller;
commissioning_robot::ControlState C_drive_lift_controller;
commissioning_robot::ControlState C_arm_controller;
commissioning_robot::ControlState C_cable_manager_controller;
commissioning_robot::FeedbackState F_pose_controller;
commissioning_robot::FeedbackState F_drive_wheel_controller;
commissioning_robot::FeedbackState F_main_lift_controller;
commissioning_robot::FeedbackState F_drive_lift_controller;
commissioning_robot::FeedbackState F_arm_controller;
commissioning_robot::FeedbackState F_cable_manager_controller;
commissioning_robot::PhaseReport PhaseReport_arm;
commissioning_robot::PhaseReport PhaseReport_cable;
std_msgs::Float32 RHWS_IMU_pitch, RHWS_IMU_pitch_biased;

class state_trigger {
    // confirmed Nov 5
   private:
    bool previous_state, is_edge_rise;

   public:
    // arg:name <= BUTTON NAME LIST (ENUM)
    state_trigger() {
        is_edge_rise = true;
    }
    ~state_trigger() {}

    bool get_trigger(bool current_state) {
        bool trigger_flag = false;
        if (previous_state != current_state) {
            if (is_edge_rise == true && current_state == true)
                trigger_flag = true;
            else if (is_edge_rise == false && current_state == false)
                trigger_flag = true;
        }

        previous_state = current_state;

        if (trigger_flag) return true;
        return false;
    }

    void set_trigger_rise() {
        is_edge_rise = true;
    }
    void set_trigger_fall() {
        is_edge_rise = false;
    }
};

state_trigger buttons_trigger[NUM_BUTTON_LIST];
state_trigger DS4_setled;

std::vector<bool> buttons;
void compose_DS4_status() {
    if (buttons.empty()) {
        buttons.resize(NUM_BUTTON_LIST);
    }
    uint8_t vec_index = 0;
    buttons.at(vec_index++) = bool(ds4_status.button_dpad_right);
    buttons.at(vec_index++) = bool(ds4_status.button_dpad_up);
    buttons.at(vec_index++) = bool(ds4_status.button_dpad_left);
    buttons.at(vec_index++) = bool(ds4_status.button_dpad_down);
    buttons.at(vec_index++) = bool(ds4_status.button_circle);
    buttons.at(vec_index++) = bool(ds4_status.button_triangle);
    buttons.at(vec_index++) = bool(ds4_status.button_square);
    buttons.at(vec_index++) = bool(ds4_status.button_cross);
    buttons.at(vec_index++) = bool(ds4_status.button_l1);
    buttons.at(vec_index++) = bool(ds4_status.button_r1);
    buttons.at(vec_index++) = bool(ds4_status.button_l2);
    buttons.at(vec_index++) = bool(ds4_status.button_r2);
    buttons.at(vec_index++) = bool(ds4_status.button_l3);
    buttons.at(vec_index++) = bool(ds4_status.button_r3);
    buttons.at(vec_index++) = bool(ds4_status.button_options);
    buttons.at(vec_index++) = bool(ds4_status.button_share);
    buttons.at(vec_index++) = bool(ds4_status.button_ps);
    buttons.at(vec_index++) = bool(ds4_status.button_trackpad);
}

/*// 手動操作用処理は一旦消去（IMUベースの制御にするため）
const double drive_lift_increment_step = 0.001;
const int8_t drive_lift_pitch_sign[NUM_DRIVEUNIT] = {1, 1, -1, -1};  //適切に機体が傾く用に符号を調整する
for (int i = 0; i < NUM_DRIVEUNIT; i++) {
    if (C_drive_lift_controller.mode[i] == CONTROL_MODE::ORIGIN) {
        C_drive_lift_controller.manual[i] = 0;  //原点モードのときはmanual制御量をゼロに設定
    } else if (C_drive_lift_controller.mode[i] == CONTROL_MODE::FOLLOW) {
        if (ds4_status.button_triangle) {
            C_drive_lift_controller.manual[i] += drive_lift_pitch_sign[i] * drive_lift_increment_step;
        }
        if (ds4_status.button_cross) {
            C_drive_lift_controller.manual[i] -= drive_lift_pitch_sign[i] * drive_lift_increment_step;
        }
    }
}
//可動域（下限）を超えた場合、他のリフトに移動量を加算してそのリフトの制御量をゼロにする。
for (int i = 0; i < NUM_DRIVEUNIT; i++) {
    if (C_drive_lift_controller.manual[i] < 0) {
        for (int j = 0; j < NUM_DRIVEUNIT; j++) {
            if (j != i)
                C_drive_lift_controller.manual[j] += -C_drive_lift_controller.manual[i];
        }
        C_drive_lift_controller.manual[i] = 0;
    }
}
for (int i = 0; i < NUM_DRIVEUNIT; i++) {
    C_drive_lift_controller.manual[i] = constrain(C_drive_lift_controller.manual[i], 0, 0.334);
}*/

/*const double main_lift_increment_step = 0.0002;
const int8_t main_lift_pitch_sign = 1;  //適切に符号を調整する
if (C_main_lift_controller.mode[axis_MAIN] == CONTROL_MODE::ORIGIN || C_main_lift_controller.reference[axis_MAIN] == REF_BOTTOM) {
    C_main_lift_controller.manual[axis_MAIN] = 0;  //原点モードのときはmanual制御量をゼロに設定
} else if (C_main_lift_controller.mode[axis_MAIN] == CONTROL_MODE::FOLLOW) {
    if (buttons.at(R1)) {
        C_main_lift_controller.manual[axis_MAIN] += main_lift_pitch_sign * main_lift_increment_step;
    }
    if (buttons.at(L1)) {
        C_main_lift_controller.manual[axis_MAIN] -= main_lift_pitch_sign * main_lift_increment_step;
    }
}*/

/*
・移動
        ・横　axis_left_x
        ・縦　axis_left_y
        ・旋回　axis_r2 & axis_l2
・足回り上下
        ・IMUで水平維持　toggle button_options
        ・手動操作　(pitch only) press button_triangle & button_cross
        ・原点固定　toggle button_options
・メイン展開上下
        ・展開有効化　toggle button_share
        ・微調整用入力　press button_l1 & button_r1
        ・原点固定　toggle button_share
・アーム部上下
        ・展開パターン保存　button_dpad_up & button_dpad_down
        ・微調整用入力　axis_right_y
        ・原点固定　button_square
・展開全原点
        ・全リセット　button_ps
*/

void process_ds4_status_to_module_input() {
    /*
     *  module-input for drive wheel controller
     *  keys : axis_left_x, axis_left_y, axis_r2, axis_l2
     */
    robot_velocity.linear.x = (-ds4_status.axis_left_x * ROBOT_WORLD_AXIS_MOVING_SPEED_MPS) + (-ds4_status.axis_right_x * ROBOT_WORLD_AXIS_MOVING_SPEED_MPS / 4.0);
    robot_velocity.linear.y = (ds4_status.axis_left_y * ROBOT_WORLD_AXIS_MOVING_SPEED_MPS) + (ds4_status.axis_right_y * ROBOT_WORLD_AXIS_MOVING_SPEED_MPS / 4.0);
    robot_velocity.angular.z = ds4_status.axis_r2 - ds4_status.axis_l2;
    drive_inverse_kinematics.linear.x = robot_velocity.linear.x;
    drive_inverse_kinematics.linear.y = robot_velocity.linear.y;
    drive_inverse_kinematics.angular.z = robot_velocity.angular.z;
    if (buttons_trigger[TRACKPAD].get_trigger(buttons.at(TRACKPAD)) && ds4_status.touch0.x < 0.5) {
        C_drive_wheel_controller.mode[0] = CONTROL_MODE::STOP;
        C_drive_wheel_controller.mode[1] = CONTROL_MODE::STOP;
        C_drive_wheel_controller.mode[2] = CONTROL_MODE::STOP;
        C_drive_wheel_controller.mode[3] = CONTROL_MODE::STOP;
    } else {
        C_drive_wheel_controller.mode[0] = CONTROL_MODE::FOLLOW;
        C_drive_wheel_controller.mode[1] = CONTROL_MODE::FOLLOW;
        C_drive_wheel_controller.mode[2] = CONTROL_MODE::FOLLOW;
        C_drive_wheel_controller.mode[3] = CONTROL_MODE::FOLLOW;
    }

    static float IMU_pitch_bias = 0;
    if (buttons_trigger[R3].get_trigger(buttons.at(R3))) {
        IMU_pitch_bias = RHWS_IMU_pitch.data;
    }
    RHWS_IMU_pitch_biased.data = RHWS_IMU_pitch.data - IMU_pitch_bias;

    /*
     *  brief : 初回時に、XYの地点情報を生成する
     *          機械的最大パルス数を超えていないかを確認
     */
    static const int16_t max_arm_pulse[NUM_ARM_AXIS] = {3900, 7300};
    static std::vector<int16_t> control_point_arm_x_pulse, control_point_arm_y_pulse;
    static constexpr uint16_t arm_y_step_pulse = 220;
    if (control_point_arm_x_pulse.empty()) {
        control_point_arm_x_pulse.push_back(0);
        control_point_arm_x_pulse.push_back(0);  // prev 500
        control_point_arm_x_pulse.push_back(REFERENCE_ARM_X);
        if (control_point_arm_x_pulse.at(control_point_arm_x_pulse.size() - 1) > max_arm_pulse[axis_X]) {
            ROS_FATAL("control_point_arm_x_pulse has OVERPULSES point(s)! recheck REFERENCE_ARM_X");
        }
    }
    if (control_point_arm_y_pulse.empty()) {
        for (uint8_t i = 0; i < 33; i++) {
            control_point_arm_y_pulse.push_back(arm_y_step_pulse * i);
        }
        if (control_point_arm_y_pulse.at(control_point_arm_y_pulse.size() - 1) > max_arm_pulse[axis_Y]) {
            ROS_FATAL("control_point_arm_y_pulse has OVERPULSES point(s)! recheck arm_y_step_pulse and reference points");
        }
    }

    /*
     *  keys  : R1, L1
     *  brief : ロボット全体のキーバインドのモードスイッチング
     */
    enum key_bind {
        AUTO_BASED,
        MANUAL_BASED,
        BIND_MODE_NUM,
    };
    static int8_t key_bind_mode = AUTO_BASED;  // BIND変更機能を消去により追加、永久的にAUTOモード
    // ケーブル敷設機構の手動モードのためBIND変更機能を消去
    // static int8_t key_bind_mode = 0;
    // if (buttons_trigger[R1].get_trigger(buttons.at(R1))) {
    //     key_bind_mode = constrain(++key_bind_mode, AUTO_BASED, BIND_MODE_NUM - 1);
    //     ds4_feedback.set_led = 1;
    //     ds4_feedback.led_r = 0;
    //     ds4_feedback.led_g = 0.5;
    //     ds4_feedback.led_b = 0.3;
    // }
    // if (buttons_trigger[L1].get_trigger(buttons.at(L1))) {
    //     key_bind_mode = constrain(--key_bind_mode, AUTO_BASED, BIND_MODE_NUM - 1);
    //     ds4_feedback.set_led = 1;
    //     ds4_feedback.led_r = 0.5;
    //     ds4_feedback.led_g = 0.3;
    //     ds4_feedback.led_b = 0;
    // }

    static int8_t arm_lift_pos_x = 0;  // 0 = origin, 1 = left, 2 = right
    static int8_t arm_lift_pos_y = 0;  // 0 = origin, <0 = step * index
    static bool enable_xy_arm = true, enable_cable_motor = true;

    static uint8_t drive_lift_mode = 0;
    if (key_bind_mode == AUTO_BASED) {
        /*
         *  module-input for drive lift controller
         *  keys  : OPTIONS
         *  brief : 足回りコントローラの動作状態のみを決定する。
         */
        if (buttons_trigger[OPTIONS].get_trigger(buttons.at(OPTIONS))) {
            drive_lift_mode++;
            if (drive_lift_mode > 2) {
                drive_lift_mode = 0;
            }
        }
        for (int i = 0; i < NUM_DRIVEUNIT; i++) {
            if (drive_lift_mode == 0) {
                C_drive_lift_controller.mode[i] = CONTROL_MODE::ORIGIN;
                C_drive_lift_controller.reference[i] = 0;
            } else if (drive_lift_mode == 1) {
                C_drive_lift_controller.mode[i] = CONTROL_MODE::FOLLOW;
                C_drive_lift_controller.reference[i] = 90;
            } else if (drive_lift_mode == 2) {
                C_drive_lift_controller.mode[i] = CONTROL_MODE::FOLLOW;
                C_drive_lift_controller.reference[i] = 350;
            }
        }

        /*
         *  module-input for main lift controller
         *  keys  : SHARE
         *  brief : メインリフトコントローラの動作状態と最終目標値を決定する。
         *  rostopic  : rostopic pub /FeedbackState/main_lift_controller commissioning_robot/FeedbackState "is_ended: [1]"
         */
        static const double REF_TOP = REFERENCE_MAIN_LIFT;  // 10/15 target:0.30m (measured : 90mm -> 385mm => 295mm accu)
        if (buttons_trigger[SHARE].get_trigger(buttons.at(SHARE))) {
            if (C_main_lift_controller.mode[axis_MAIN] == CONTROL_MODE::ORIGIN || C_main_lift_controller.mode[axis_MAIN] == CONTROL_MODE::STOP) {
                C_main_lift_controller.mode[axis_MAIN] = CONTROL_MODE::FOLLOW;
                C_main_lift_controller.reference[axis_MAIN] = REF_TOP;  //(0.55 * 4)
            } else {
                C_main_lift_controller.mode[axis_MAIN] = CONTROL_MODE::ORIGIN;
                C_main_lift_controller.reference[axis_MAIN] = 0;
                C_main_lift_controller.manual[axis_MAIN] = 0;
            }
        }

        static uint8_t cable_manager_autorun_macro_phase = 0, cable_manager_push_step_number = 0;
        // ケーブル敷設機構を有効化、ケーブルリング敷設シーケンスの実行フラグを立てる
        if (buttons_trigger[CIRCLE].get_trigger(buttons.at(CIRCLE))) {
            enable_cable_motor = true;
            cable_manager_autorun_macro_phase = 1;
        }
        // ケーブル敷設機構を有効化、上下ユニット・ベルトユニットを原点回帰モードに設定
        // ベルトユニットのパルス計算用インデックスをリセットする（動作中にむやみに原点を取らないように注意）
        if (buttons_trigger[SQUARE].get_trigger(buttons.at(SQUARE))) {
            enable_cable_motor = true;
            C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::ORIGIN;
            C_cable_manager_controller.mode[BELT_MOTOR] = CONTROL_MODE::ORIGIN;
            C_cable_manager_controller.reference[ARM_MOTOR] = 0;
            C_cable_manager_controller.reference[BELT_MOTOR] = 0;
            cable_manager_push_step_number = 0;
            cable_manager_autorun_macro_phase = 0;
        }
        // ケーブル敷設機構を無効化
        if (buttons_trigger[CROSS].get_trigger(buttons.at(CROSS))) {
            enable_cable_motor = false;
        }

        // uint32_t a = ros::Time::now().sec;
        // uint32_t b = ros::Time::now().nsec;
        // ROS_INFO("a:%d", a);
        // ROS_INFO("b:%d", b);
        // static double c, d;

        // c = double(double(a) + (double(b) / 1e9));
        // ROS_INFO("c-d:%lf", c - d);
        // d = c;
        // static double start_time = 0;

        ROS_INFO("##### cable_manager_autorun_macro_phase #####");
        ROS_INFO("cable_manager_autorun_macro_phase:%d", cable_manager_autorun_macro_phase);
        PhaseReport_cable.phase = cable_manager_autorun_macro_phase;
        switch (cable_manager_autorun_macro_phase) {
            // ブレイク
            case 0:
                PhaseReport_cable.description = "STALE";
                break;

            // 上下ユニットを原点回帰モードに設定（事前にベルトユニットの原点合わせを行うこと（四角ボタン））
            case 1:
                ROS_INFO("F_cable_manager_controller.is_ended.at(ARM_MOTOR):%d", F_cable_manager_controller.is_ended.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR));
                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::ORIGIN;
                C_cable_manager_controller.reference[ARM_MOTOR] = 0;

                drive_lift_mode = 0;  // 足回り上下を原点モードに設定

                if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::ORIGIN) {
                    cable_manager_autorun_macro_phase = 2;
                }
                PhaseReport_cable.description = "原点取得中";
                break;

            // ベルトユニットを追従動作モードに設定、目標値は(1個分のパルス数×インデックス)＋1つ目のオフセット
            case 2:
                ROS_INFO("F_cable_manager_controller.is_ended.at(BELT_MOTOR):%d", F_cable_manager_controller.is_ended.at(BELT_MOTOR));
                ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR));
                ROS_INFO("F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR):%lf", F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR));
                C_cable_manager_controller.mode[BELT_MOTOR] = CONTROL_MODE::FOLLOW;
                C_cable_manager_controller.reference[BELT_MOTOR] = (REFERENCE_CABLE_STEP_BELT_MOTOR * cable_manager_push_step_number) + REFERENCE_CABLE_FIRST_BELT_MOTOR;

                if (F_cable_manager_controller.is_ended.at(BELT_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR) == CONTROL_MODE::FOLLOW &&
                    F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR) == C_cable_manager_controller.reference[BELT_MOTOR]) {
                    cable_manager_autorun_macro_phase = 3;
                    cable_manager_push_step_number++;  // 手動操作実装による追加
                }
                PhaseReport_cable.description = "ベルト位置決め中";
                break;

            case 3:
                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::MANUAL;
                if (buttons.at(R1)) {
                    C_cable_manager_controller.manual[ARM_MOTOR] = FOLLOW_PWM_CABLE_ARM_MOTOR;
                } else if (buttons.at(L1)) {
                    C_cable_manager_controller.manual[ARM_MOTOR] = -FOLLOW_PWM_CABLE_ARM_MOTOR;
                } else {
                    C_cable_manager_controller.manual[ARM_MOTOR] = 0;
                }
                if (buttons_trigger[TRIANGLE].get_trigger(buttons.at(TRIANGLE))) {
                    cable_manager_autorun_macro_phase = 5;
                }
                PhaseReport_cable.description = "上下操作受付中";
                break;

            // // 上下ユニットを追従動作モードに設定、目標値はケーブルリングが地面に接地するまでのパルス数
            // case 3:
            //     ROS_INFO("F_cable_manager_controller.is_ended.at(ARM_MOTOR):%d", F_cable_manager_controller.is_ended.at(ARM_MOTOR));
            //     ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR));
            //     ROS_INFO("F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR):%lf", F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR));

            //     C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::FOLLOW;
            //     C_cable_manager_controller.reference[ARM_MOTOR] = REFERENCE_CABLE_ARM_MOTOR;

            //     if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
            //         F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::FOLLOW &&
            //         F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR) == C_cable_manager_controller.reference[ARM_MOTOR]) {
            //         // cable_manager_autorun_macro_phase = 4;
            //         cable_manager_autorun_macro_phase = 5;
            //         cable_manager_push_step_number++;  // case 4の消去に伴う追加！　case 4戻す際は消去すること！
            //     }
            //     PhaseReport_cable.description = "支点ユニット押下中";
            //     break;

            // // ベルトユニットを追従動作モードに設定、目標値は前回設定値-30パルス（少し戻す）（目的はケーブルリングをマグネットから確実に外すこと）
            // case 4:
            //     ROS_INFO("F_cable_manager_controller.is_ended.at(BELT_MOTOR):%d", F_cable_manager_controller.is_ended.at(BELT_MOTOR));
            //     ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR));
            //     ROS_INFO("F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR):%lf", F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR));

            //     C_cable_manager_controller.mode[BELT_MOTOR] = CONTROL_MODE::FOLLOW;
            //     C_cable_manager_controller.reference[BELT_MOTOR] = (REFERENCE_CABLE_STEP_BELT_MOTOR * cable_manager_push_step_number) + REFERENCE_CABLE_FIRST_BELT_MOTOR + 50;

            //     if (F_cable_manager_controller.is_ended.at(BELT_MOTOR) &&
            //         F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR) == CONTROL_MODE::FOLLOW &&
            //         F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR) == C_cable_manager_controller.reference[BELT_MOTOR]) {
            //         cable_manager_autorun_macro_phase = 5;
            //         cable_manager_push_step_number++;
            //     }
            //     break;

            // 上下ユニットを追従動作モードに設定、目標値は原点位置
            case 5:
                ROS_INFO("F_cable_manager_controller.is_ended.at(ARM_MOTOR):%d", F_cable_manager_controller.is_ended.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR):%lf", F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR));
                ROS_INFO("  ");

                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::ORIGIN;
                C_cable_manager_controller.reference[ARM_MOTOR] = 0;

                drive_lift_mode = 1;  // 足回り上下を追従モードに、高さを段階１に設定

                if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::ORIGIN &&
                    F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR) == C_cable_manager_controller.reference[ARM_MOTOR]) {
                    cable_manager_autorun_macro_phase = 0;
                }
                PhaseReport_cable.description = "初期位置回帰";
                break;
        }

        // ケーブルリングが５個だから、"１つ目の特殊オフセット"を考慮して１つ分のパルス数をインクリメントしていくのは"４"回まで。➜ ５以上にならないようにオーバーフロー検出
        if (cable_manager_push_step_number > 4) {
            cable_manager_push_step_number = 0;
        }

        // ケーブル機構が無効化されているとき
        if (enable_cable_motor == false) {
            // モーター出力を無条件に停止し、リファレンスをゼロに設定
            C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::STOP;
            C_cable_manager_controller.mode[BELT_MOTOR] = CONTROL_MODE::STOP;
            C_cable_manager_controller.reference[ARM_MOTOR] = 0;
            C_cable_manager_controller.reference[BELT_MOTOR] = 0;
        }

        static uint8_t arm_autorun_macro_phase = 0;
        static uint8_t arm_centralize_macro_phase = 0;
        static uint8_t xy_loop_phase = 0, y_point_index = 0;

        if (buttons_trigger[ARROW_LEFT].get_trigger(buttons.at(ARROW_LEFT))) {
            enable_xy_arm = true;
            C_arm_controller.mode.at(axis_X) = CONTROL_MODE::ORIGIN;
            C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::ORIGIN;
            C_arm_controller.reference.at(axis_X) = 0;
            C_arm_controller.reference.at(axis_Y) = 0;
            arm_autorun_macro_phase = 0;
            arm_centralize_macro_phase = 0;
        }
        if (buttons_trigger[ARROW_RIGHT].get_trigger(buttons.at(ARROW_RIGHT))) {
            enable_xy_arm = true;
            C_arm_controller.mode.at(axis_X) = CONTROL_MODE::FAST_FOLLOW;
            C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::FAST_FOLLOW;
            C_arm_controller.reference.at(axis_X) = 0;
            C_arm_controller.reference.at(axis_Y) = 0;
            arm_autorun_macro_phase = 0;
            arm_centralize_macro_phase = 0;
        }
        if (buttons_trigger[ARROW_DOWN].get_trigger(buttons.at(ARROW_DOWN))) {
            enable_xy_arm = false;
        }

        if (buttons_trigger[ARROW_UP].get_trigger(buttons.at(ARROW_UP))) {
            enable_xy_arm = true;
            arm_autorun_macro_phase = 1;  // enter process with pressing the button
            arm_centralize_macro_phase = 0;
            xy_loop_phase = 0;
            y_point_index = 0;
        }

        PhaseReport_arm.phase = arm_autorun_macro_phase;
        switch (arm_autorun_macro_phase) {
            case 0:
                PhaseReport_arm.description = "STALE";
                break;
            case 1:  //一旦リセットを挟んでおく（この前に高速リセットで原点付近にいるはず）
                C_arm_controller.mode.at(axis_X) = CONTROL_MODE::ORIGIN;
                C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::ORIGIN;
                C_arm_controller.reference.at(axis_X) = 0;
                C_arm_controller.reference.at(axis_Y) = 0;
                if (F_arm_controller.is_ended.at(axis_X) &&
                    F_arm_controller.is_ended.at(axis_Y) &&
                    F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::ORIGIN &&
                    F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::ORIGIN)
                    arm_autorun_macro_phase = 2;
                PhaseReport_arm.description = "原点取得中";
                break;
            case 2:  //追従モードとリファレンスが0になっていることを確認
                C_arm_controller.mode.at(axis_X) = CONTROL_MODE::FOLLOW;
                C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::FOLLOW;
                C_arm_controller.reference.at(axis_X) = 0;
                C_arm_controller.reference.at(axis_Y) = 0;
                if (F_arm_controller.reference_feedbackside.at(axis_X) == 0 &&
                    F_arm_controller.reference_feedbackside.at(axis_Y) == 0 &&
                    F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::FOLLOW &&
                    F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::FOLLOW &&
                    F_arm_controller.is_ended.at(axis_X) &&
                    F_arm_controller.is_ended.at(axis_Y))
                    arm_autorun_macro_phase = 3;
                PhaseReport_arm.description = "追従モード設定中";
                break;
            case 3:
                if (xy_loop_phase == 0) {
                    C_arm_controller.reference.at(axis_X) = control_point_arm_x_pulse.at(2);  // away
                    C_arm_controller.reference.at(axis_Y) = control_point_arm_y_pulse.at(y_point_index);
                    if (F_arm_controller.reference_feedbackside.at(axis_X) == control_point_arm_x_pulse.at(2) &&
                        F_arm_controller.reference_feedbackside.at(axis_Y) == control_point_arm_y_pulse.at(y_point_index) &&
                        F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.is_ended.at(axis_X) &&
                        F_arm_controller.is_ended.at(axis_Y)) {
                        xy_loop_phase = 1;
                        y_point_index++;
                    }
                    PhaseReport_arm.description = "X軸移動中";
                } else if (xy_loop_phase == 1) {
                    C_arm_controller.reference.at(axis_X) = control_point_arm_x_pulse.at(2);  // away
                    C_arm_controller.reference.at(axis_Y) = control_point_arm_y_pulse.at(y_point_index);
                    if (F_arm_controller.reference_feedbackside.at(axis_X) == control_point_arm_x_pulse.at(2) &&
                        F_arm_controller.reference_feedbackside.at(axis_Y) == control_point_arm_y_pulse.at(y_point_index) &&
                        F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.is_ended.at(axis_X) &&
                        F_arm_controller.is_ended.at(axis_Y)) {
                        xy_loop_phase = 2;
                    }
                    PhaseReport_arm.description = "Y軸移動中";
                } else if (xy_loop_phase == 2) {
                    C_arm_controller.reference.at(axis_X) = control_point_arm_x_pulse.at(1);  // origin
                    C_arm_controller.reference.at(axis_Y) = control_point_arm_y_pulse.at(y_point_index);
                    if (F_arm_controller.reference_feedbackside.at(axis_X) == control_point_arm_x_pulse.at(1) &&
                        F_arm_controller.reference_feedbackside.at(axis_Y) == control_point_arm_y_pulse.at(y_point_index) &&
                        F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.is_ended.at(axis_X) &&
                        F_arm_controller.is_ended.at(axis_Y)) {
                        xy_loop_phase = 3;
                        y_point_index++;
                    }
                    PhaseReport_arm.description = "X軸移動中";
                } else if (xy_loop_phase == 3) {
                    C_arm_controller.reference.at(axis_X) = control_point_arm_x_pulse.at(1);  // origin
                    C_arm_controller.reference.at(axis_Y) = control_point_arm_y_pulse.at(y_point_index);
                    if (F_arm_controller.reference_feedbackside.at(axis_X) == control_point_arm_x_pulse.at(1) &&
                        F_arm_controller.reference_feedbackside.at(axis_Y) == control_point_arm_y_pulse.at(y_point_index) &&
                        F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::FOLLOW &&
                        F_arm_controller.is_ended.at(axis_X) &&
                        F_arm_controller.is_ended.at(axis_Y)) {
                        xy_loop_phase = 0;
                    }
                    PhaseReport_arm.description = "Y軸移動中";
                }
                ROS_INFO_THROTTLE(1, "y_point_index:%d", y_point_index);
                if (control_point_arm_y_pulse.size() - 1 < y_point_index) {
                    y_point_index = 0;
                    arm_autorun_macro_phase = 0;
                    PhaseReport_arm.description = "塗りつぶし完了";
                }
                break;
        }

        if (buttons_trigger[PS].get_trigger(buttons.at(PS))) {
            enable_xy_arm = true;
            arm_autorun_macro_phase = 0;
            arm_centralize_macro_phase = 1;  // enter process with pressing the button
        }
        switch (arm_centralize_macro_phase) {
            case 0:
                break;
            case 1:
                ROS_INFO("F_arm_controller.is_ended.at(axis_X):%d", F_arm_controller.is_ended.at(axis_X));
                ROS_INFO("F_arm_controller.is_ended.at(axis_Y):%d", F_arm_controller.is_ended.at(axis_Y));
                ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_X):%d", F_arm_controller.mode_feedbackside.at(axis_X));
                ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_Y):%d", F_arm_controller.mode_feedbackside.at(axis_Y));
                C_arm_controller.mode.at(axis_X) = CONTROL_MODE::ORIGIN;
                C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::ORIGIN;
                C_arm_controller.reference.at(axis_X) = 0;
                C_arm_controller.reference.at(axis_Y) = 0;
                if (F_arm_controller.is_ended.at(axis_X) &&
                    F_arm_controller.is_ended.at(axis_Y) &&
                    F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::ORIGIN &&
                    F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::ORIGIN)
                    arm_centralize_macro_phase = 2;
                PhaseReport_arm.description = "原点取得中";
                break;
            case 2:
                ROS_INFO("F_arm_controller.reference_feedbackside.at(axis_X):%lf", F_arm_controller.reference_feedbackside.at(axis_X));
                ROS_INFO("F_arm_controller.reference_feedbackside.at(axis_Y):%lf", F_arm_controller.reference_feedbackside.at(axis_Y));
                ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_X):%d", F_arm_controller.mode_feedbackside.at(axis_X));
                ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_Y):%d", F_arm_controller.mode_feedbackside.at(axis_Y));
                ROS_INFO("F_arm_controller.is_ended.at(axis_X):%d", F_arm_controller.is_ended.at(axis_X));
                ROS_INFO("F_arm_controller.is_ended.at(axis_Y):%d", F_arm_controller.is_ended.at(axis_Y));
                C_arm_controller.mode.at(axis_X) = CONTROL_MODE::FAST_FOLLOW;
                C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::FAST_FOLLOW;
                C_arm_controller.reference.at(axis_X) = (REFERENCE_ARM_X / 2);
                C_arm_controller.reference.at(axis_Y) = 3000;  // photo confirmed with members 11/5
                if (F_arm_controller.reference_feedbackside.at(axis_X) == (REFERENCE_ARM_X / 2) &&
                    F_arm_controller.reference_feedbackside.at(axis_Y) == 3000 &&
                    F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::FAST_FOLLOW &&
                    F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::FAST_FOLLOW &&
                    F_arm_controller.is_ended.at(axis_X) &&
                    F_arm_controller.is_ended.at(axis_Y))
                    arm_centralize_macro_phase = 0;
                PhaseReport_arm.description = "高速中央化中";
                break;
        }

        if (enable_xy_arm == false) {
            C_arm_controller.mode[axis_X] = CONTROL_MODE::STOP;
            C_arm_controller.mode[axis_Y] = CONTROL_MODE::STOP;
            C_arm_controller.reference[axis_X] = 0;
            C_arm_controller.reference[axis_Y] = 0;
        }

    } else if (key_bind_mode == MANUAL_BASED) {
        /*
         *  module-input for arm lift controller
         *  keys  : axis_left_x, axis_left_y, axis_r2, axis_l2
         *  brief : 足回りコントローラの動作状態のみを決定する。
         */
        if (buttons_trigger[CIRCLE].get_trigger(buttons.at(CIRCLE))) {
            enable_xy_arm = true;
        }
        if (buttons_trigger[CROSS].get_trigger(buttons.at(CROSS))) {
            enable_xy_arm = false;
        }

        static const uint8_t control_point_arm_x = control_point_arm_x_pulse.size() - 1;
        static const uint8_t control_point_arm_y = control_point_arm_y_pulse.size() - 1;

        if (buttons_trigger[ARROW_RIGHT].get_trigger(buttons.at(ARROW_RIGHT))) {
            arm_lift_pos_x = constrain(++arm_lift_pos_x, 0, control_point_arm_x);
        }
        if (buttons_trigger[ARROW_LEFT].get_trigger(buttons.at(ARROW_LEFT))) {
            arm_lift_pos_x = constrain(--arm_lift_pos_x, 0, control_point_arm_x);
        }
        if (buttons_trigger[ARROW_UP].get_trigger(buttons.at(ARROW_UP))) {
            arm_lift_pos_y = constrain(++arm_lift_pos_y, 0, control_point_arm_y);
        }
        if (buttons_trigger[ARROW_DOWN].get_trigger(buttons.at(ARROW_DOWN))) {
            arm_lift_pos_y = constrain(--arm_lift_pos_y, 0, control_point_arm_y);
        }

        if (enable_xy_arm) {
            if (arm_lift_pos_x == 0) {
                C_arm_controller.mode[axis_X] = CONTROL_MODE::ORIGIN;
                C_arm_controller.reference[axis_X] = 0;
            } else {
                C_arm_controller.mode[axis_X] = CONTROL_MODE::FOLLOW;
                C_arm_controller.reference[axis_X] = control_point_arm_x_pulse.at(arm_lift_pos_x);
            }

            if (arm_lift_pos_y == 0) {
                C_arm_controller.mode[axis_Y] = CONTROL_MODE::ORIGIN;
                C_arm_controller.reference[axis_Y] = 0;
            } else {
                C_arm_controller.mode[axis_Y] = CONTROL_MODE::FOLLOW;
                C_arm_controller.reference[axis_Y] = control_point_arm_y_pulse.at(arm_lift_pos_y);
            }
        } else {
            C_arm_controller.mode[axis_X] = CONTROL_MODE::STOP;
            C_arm_controller.mode[axis_Y] = CONTROL_MODE::STOP;
            C_arm_controller.reference[axis_X] = 0;
            C_arm_controller.reference[axis_Y] = 0;
        }
    }

    //ケーブル敷設機構の手動モード実装のためmain_lift_enable機能を消去
    // static bool main_lift_enable = true;
    // if (buttons_trigger[TRIANGLE].get_trigger(buttons.at(TRIANGLE))) {
    //     main_lift_enable = !main_lift_enable;
    // }
    // if (!main_lift_enable) {
    //     C_main_lift_controller.mode.at(axis_MAIN) = CONTROL_MODE::STOP;
    // }
}

//速度制御ノードにホイールの目標速度を送信するためにデータ格納
void callback_drive_inverse_kinematics(const std_msgs::Float32MultiArray &msg) {
    if (msg.data.size() != NUM_DRIVEUNIT)
        return;

    for (int i = 0; i < NUM_DRIVEUNIT; i++) {
        C_drive_wheel_controller.mode[i] = CONTROL_MODE::FOLLOW;
        C_drive_wheel_controller.reference[i] = msg.data[i];
    }
}

void callback_ds4_driver(const ds4_driver::Status &msg) {
    ds4_status = msg;
}
void callback_pose_controller(const commissioning_robot::FeedbackState &msg) {
    F_pose_controller = msg;
}
void callback_drive_wheel_controller(const commissioning_robot::FeedbackState &msg) {
    F_drive_wheel_controller = msg;
}
void callback_main_lift_controller(const commissioning_robot::FeedbackState &msg) {
    F_main_lift_controller = msg;
}
void callback_drive_lift_controller(const commissioning_robot::FeedbackState &msg) {
    F_drive_lift_controller = msg;
}
void callback_arm_controller(const commissioning_robot::FeedbackState &msg) {
    F_arm_controller = msg;
}
void callback_cable_manager_controller(const commissioning_robot::FeedbackState &msg) {
    F_cable_manager_controller = msg;
}
void callback_RHWS_IMU_pitch(const std_msgs::Float32 &msg) {
    RHWS_IMU_pitch.data = msg.data;
}

int main(int argv, char **argc) {
    ros::init(argv, argc, "ROBOT_CORE");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Publisher pub_Twist_drive_inverse_kinematics = nh.advertise<geometry_msgs::Twist>("drive_inverse_kinematics_Twist", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_ControlState_pose_controller = nh.advertise<commissioning_robot::ControlState>("ControlState/pose_controller", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_ControlState_drive_wheel_controller = nh.advertise<commissioning_robot::ControlState>("ControlState/drive_wheel_controller", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_ControlState_main_lift_controller = nh.advertise<commissioning_robot::ControlState>("ControlState/main_lift_controller", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_ControlState_drive_lift_controller = nh.advertise<commissioning_robot::ControlState>("ControlState/drive_lift_controller", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_ControlState_arm_controller = nh.advertise<commissioning_robot::ControlState>("ControlState/arm_controller", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_ControlState_cable_manager_controller = nh.advertise<commissioning_robot::ControlState>("ControlState/cable_manager_controller", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_PhaseReport_arm_controller = nh.advertise<commissioning_robot::PhaseReport>("PhaseReport/arm_controller", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_PhaseReport_cable_manager_controller = nh.advertise<commissioning_robot::PhaseReport>("PhaseReport/cable_manager_controller", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_ds4_driver = nh.advertise<ds4_driver::Feedback>("set_feedback", QUEUE_SIZE_PUBLISHER);
    ros::Subscriber sub_ds4_driver = nh.subscribe("status", QUEUE_SIZE_SUBSCRIBER, callback_ds4_driver);

    ros::Publisher pub_RHWS_IMU_pitch_biased = nh.advertise<std_msgs::Float32>("RHWS/IMU_pitch_biased", QUEUE_SIZE_PUBLISHER);
    ros::Subscriber sub_RHWS_IMU_pitch = nh.subscribe("RHWS/IMU_pitch", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_IMU_pitch);

    ros::Subscriber sub_drive_inverse_kinematics = nh.subscribe("inverse_kinematics_drive_wheels_velocity", QUEUE_SIZE_SUBSCRIBER, callback_drive_inverse_kinematics);
    ros::Subscriber sub_FeedbackState_pose_controller = nh.subscribe("FeedbackState/pose_controller", QUEUE_SIZE_SUBSCRIBER, callback_pose_controller);
    ros::Subscriber sub_FeedbackState_drive_wheel_controller = nh.subscribe("FeedbackState/drive_wheel_controller", QUEUE_SIZE_SUBSCRIBER, callback_drive_wheel_controller);
    ros::Subscriber sub_FeedbackState_main_lift_controller = nh.subscribe("FeedbackState/main_lift_controller", QUEUE_SIZE_SUBSCRIBER, callback_main_lift_controller);
    ros::Subscriber sub_FeedbackState_drive_lift_controller = nh.subscribe("FeedbackState/drive_lift_controller", QUEUE_SIZE_SUBSCRIBER, callback_drive_lift_controller);
    ros::Subscriber sub_FeedbackState_arm_controller = nh.subscribe("FeedbackState/arm_controller", QUEUE_SIZE_SUBSCRIBER, callback_arm_controller);
    ros::Subscriber sub_FeedbackState_cable_manager_controller = nh.subscribe("FeedbackState/cable_manager_controller", QUEUE_SIZE_SUBSCRIBER, callback_cable_manager_controller);

    C_drive_wheel_controller.mode.resize(NUM_DRIVEUNIT);
    C_drive_wheel_controller.reference.resize(NUM_DRIVEUNIT);

    C_drive_lift_controller.mode.resize(NUM_DRIVEUNIT);
    C_drive_lift_controller.reference.resize(NUM_DRIVEUNIT);
    C_drive_lift_controller.manual.resize(NUM_DRIVEUNIT);
    C_drive_lift_controller.offset.resize(NUM_DRIVEUNIT);
    F_drive_lift_controller.current.resize(NUM_DRIVEUNIT);
    F_drive_lift_controller.is_ended.resize(NUM_DRIVEUNIT);
    F_drive_lift_controller.reference_feedbackside.resize(NUM_DRIVEUNIT);
    F_drive_lift_controller.mode_feedbackside.resize(NUM_DRIVEUNIT);

    C_main_lift_controller.mode.resize(NUM_MAIN_LIFT_AXIS);
    C_main_lift_controller.reference.resize(NUM_MAIN_LIFT_AXIS);
    C_main_lift_controller.manual.resize(NUM_MAIN_LIFT_AXIS);
    C_main_lift_controller.offset.resize(NUM_MAIN_LIFT_AXIS);
    F_main_lift_controller.current.resize(NUM_MAIN_LIFT_AXIS);
    F_main_lift_controller.is_ended.resize(NUM_MAIN_LIFT_AXIS);
    F_main_lift_controller.reference_feedbackside.resize(NUM_MAIN_LIFT_AXIS);
    F_main_lift_controller.mode_feedbackside.resize(NUM_MAIN_LIFT_AXIS);

    C_arm_controller.mode.resize(NUM_ARM_AXIS);
    C_arm_controller.reference.resize(NUM_ARM_AXIS);
    C_arm_controller.manual.resize(NUM_ARM_AXIS);
    C_arm_controller.offset.resize(NUM_ARM_AXIS);
    F_arm_controller.current.resize(NUM_ARM_AXIS);
    F_arm_controller.is_ended.resize(NUM_ARM_AXIS);
    F_arm_controller.reference_feedbackside.resize(NUM_ARM_AXIS);
    F_arm_controller.mode_feedbackside.resize(NUM_ARM_AXIS);

    C_cable_manager_controller.mode.resize(NUM_CABLE_MOTOR);
    C_cable_manager_controller.reference.resize(NUM_CABLE_MOTOR);
    C_cable_manager_controller.manual.resize(NUM_CABLE_MOTOR);
    C_cable_manager_controller.offset.resize(NUM_CABLE_MOTOR);
    F_cable_manager_controller.current.resize(NUM_CABLE_MOTOR);
    F_cable_manager_controller.is_ended.resize(NUM_CABLE_MOTOR);
    F_cable_manager_controller.reference_feedbackside.resize(NUM_CABLE_MOTOR);
    F_cable_manager_controller.mode_feedbackside.resize(NUM_CABLE_MOTOR);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        compose_DS4_status();
        process_ds4_status_to_module_input();
        if (DS4_setled.get_trigger(bool(ds4_feedback.set_led))) {
            pub_ds4_driver.publish(ds4_feedback);
            ds4_feedback.set_led = 0;
        }

        pub_Twist_drive_inverse_kinematics.publish(drive_inverse_kinematics);
        pub_ControlState_pose_controller.publish(C_pose_controller);
        pub_ControlState_drive_wheel_controller.publish(C_drive_wheel_controller);
        pub_ControlState_main_lift_controller.publish(C_main_lift_controller);
        pub_ControlState_drive_lift_controller.publish(C_drive_lift_controller);
        pub_ControlState_arm_controller.publish(C_arm_controller);
        pub_ControlState_cable_manager_controller.publish(C_cable_manager_controller);
        pub_PhaseReport_arm_controller.publish(PhaseReport_arm);
        pub_PhaseReport_cable_manager_controller.publish(PhaseReport_cable);
        pub_RHWS_IMU_pitch_biased.publish(RHWS_IMU_pitch_biased);
    }
}

/*
    static auto y_arm_origin_phase = 0;
    if (buttons_trigger[PS].get_trigger(buttons.at(PS))) {
        if (y_arm_origin_phase != 0)
            y_arm_origin_phase = 0;  // reset when pressed multitimes
        else
            y_arm_origin_phase = 2;  // enter process with pressing the button
    }

    ROS_INFO("y_arm_origin_phase:%d", y_arm_origin_phase);
    switch (y_arm_origin_phase) {
        case 0:
            break;
        // case 1:
        //     ROS_INFO("F_main_lift_controller.reference_feedbackside.at(axis_MAIN):%lf", F_main_lift_controller.reference_feedbackside.at(axis_MAIN));
        //     ROS_INFO("F_main_lift_controller.mode_feedbackside.at(axis_MAIN):%d", F_main_lift_controller.mode_feedbackside.at(axis_MAIN));
        //     ROS_INFO("F_main_lift_controller.is_ended.at(axis_MAIN):%d", F_main_lift_controller.is_ended.at(axis_MAIN));
        //     C_main_lift_controller.mode.at(axis_MAIN) = (CONTROL_MODE::FOLLOW);
        //     C_main_lift_controller.reference.at(axis_MAIN) = 0.25;  // 20cm
        //     if (F_main_lift_controller.reference_feedbackside.at(axis_MAIN) == 0.25 &&
        //         F_main_lift_controller.mode_feedbackside.at(axis_MAIN) == CONTROL_MODE::FOLLOW &&
        //         F_main_lift_controller.is_ended.at(axis_MAIN))
        //         y_arm_origin_phase = 2;
        //     break;
        case 2:
            ROS_INFO("F_arm_controller.is_ended.at(axis_X):%d", F_arm_controller.is_ended.at(axis_X));
            ROS_INFO("F_arm_controller.is_ended.at(axis_Y):%d", F_arm_controller.is_ended.at(axis_Y));
            ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_X):%d", F_arm_controller.mode_feedbackside.at(axis_X));
            ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_Y):%d", F_arm_controller.mode_feedbackside.at(axis_Y));
            C_arm_controller.mode.at(axis_X) = CONTROL_MODE::ORIGIN;
            C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::ORIGIN;
            if (F_arm_controller.is_ended.at(axis_X) &&
                F_arm_controller.is_ended.at(axis_Y) &&
                F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::ORIGIN &&
                F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::ORIGIN)
                y_arm_origin_phase = 3;
            break;
        case 3:
            ROS_INFO("F_arm_controller.reference_feedbackside.at(axis_X):%lf", F_arm_controller.reference_feedbackside.at(axis_X));
            ROS_INFO("F_arm_controller.reference_feedbackside.at(axis_Y):%lf", F_arm_controller.reference_feedbackside.at(axis_Y));
            ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_X):%d", F_arm_controller.mode_feedbackside.at(axis_X));
            ROS_INFO("F_arm_controller.mode_feedbackside.at(axis_Y):%d", F_arm_controller.mode_feedbackside.at(axis_Y));
            ROS_INFO("F_arm_controller.is_ended.at(axis_X):%d", F_arm_controller.is_ended.at(axis_X));
            ROS_INFO("F_arm_controller.is_ended.at(axis_Y):%d", F_arm_controller.is_ended.at(axis_Y));
            C_arm_controller.mode.at(axis_X) = CONTROL_MODE::FOLLOW;
            C_arm_controller.mode.at(axis_Y) = CONTROL_MODE::FOLLOW;
            C_arm_controller.reference.at(axis_X) = (REFERENCE_ARM_X / 2);
            C_arm_controller.reference.at(axis_Y) = 3000;  // photo confirmed with members 11/5
            if (F_arm_controller.reference_feedbackside.at(axis_X) == (REFERENCE_ARM_X / 2) &&
                F_arm_controller.reference_feedbackside.at(axis_Y) == 3000 &&
                F_arm_controller.mode_feedbackside.at(axis_X) == CONTROL_MODE::FOLLOW &&
                F_arm_controller.mode_feedbackside.at(axis_Y) == CONTROL_MODE::FOLLOW &&
                F_arm_controller.is_ended.at(axis_X) &&
                F_arm_controller.is_ended.at(axis_Y))
                y_arm_origin_phase = 0;
            break;
            // case 4:
            //     ROS_INFO("F_main_lift_controller.is_ended.at(axis_MAIN):%d", F_main_lift_controller.is_ended.at(axis_MAIN));
            //     ROS_INFO("F_main_lift_controller.mode_feedbackside.at(axis_MAIN):%d", F_main_lift_controller.mode_feedbackside.at(axis_MAIN));
            //     C_main_lift_controller.mode.at(axis_MAIN) = CONTROL_MODE::ORIGIN;
            //     C_main_lift_controller.reference.at(axis_MAIN) = 0.0;
            //     if (F_main_lift_controller.is_ended.at(axis_MAIN) &&
            //         F_main_lift_controller.mode_feedbackside.at(axis_MAIN) == CONTROL_MODE::ORIGIN)
            //         y_arm_origin_phase = 0;  // reset phase
            //     break;
    }
*/

/*
        static double start_time = 0;

        switch (cable_manager_autorun_macro_phase) {
            case 0:
                break;
            case 1:
                ROS_INFO("F_cable_manager_controller.is_ended.at(ARM_MOTOR):%d", F_cable_manager_controller.is_ended.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR));
                ROS_INFO("  ");
                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::ORIGIN;
                C_cable_manager_controller.reference[ARM_MOTOR] = 0;

                if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::ORIGIN) {
                    cable_manager_autorun_macro_phase = 2;
                }
                break;
            case 2:
                ROS_INFO("F_cable_manager_controller.is_ended.at(BELT_MOTOR):%d", F_cable_manager_controller.is_ended.at(BELT_MOTOR));
                ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR));
                ROS_INFO("F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR):%lf", F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR));
                ROS_INFO("  ");

                C_cable_manager_controller.mode[BELT_MOTOR] = CONTROL_MODE::FOLLOW;
                C_cable_manager_controller.reference[BELT_MOTOR] = (REFERENCE_CABLE_STEP_BELT_MOTOR * cable_manager_push_step_number) + REFERENCE_CABLE_FIRST_BELT_MOTOR;

                if (F_cable_manager_controller.is_ended.at(BELT_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR) == CONTROL_MODE::FOLLOW &&
                    F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR) == C_cable_manager_controller.reference[BELT_MOTOR]) {
                    cable_manager_autorun_macro_phase = 3;
                }
                break;
            case 3:
                ROS_INFO("F_cable_manager_controller.is_ended.at(ARM_MOTOR):%d", F_cable_manager_controller.is_ended.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR):%lf", F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR));
                ROS_INFO("  ");

                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::FOLLOW;
                C_cable_manager_controller.reference[ARM_MOTOR] = REFERENCE_CABLE_ARM_MOTOR;

                if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::FOLLOW &&
                    F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR) == C_cable_manager_controller.reference[ARM_MOTOR]) {
                    cable_manager_autorun_macro_phase = 4;
                }
                break;

            case 4:
                C_cable_manager_controller.mode[BELT_MOTOR] = CONTROL_MODE::FOLLOW;
                C_cable_manager_controller.reference[BELT_MOTOR] = (REFERENCE_CABLE_STEP_BELT_MOTOR * cable_manager_push_step_number) + REFERENCE_CABLE_FIRST_BELT_MOTOR - 30;
                if (F_cable_manager_controller.is_ended.at(BELT_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(BELT_MOTOR) == CONTROL_MODE::FOLLOW &&
                    F_cable_manager_controller.reference_feedbackside.at(BELT_MOTOR) == C_cable_manager_controller.reference[BELT_MOTOR]) {
                    cable_manager_autorun_macro_phase = 5;
                    cable_manager_push_step_number++;
                }
                break;

            case 5:
                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::FOLLOW;
                C_cable_manager_controller.reference[ARM_MOTOR] = REFERENCE_CABLE_ARM_MOTOR - 20;

                if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::FOLLOW &&
                    F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR) == C_cable_manager_controller.reference[ARM_MOTOR]) {
                    cable_manager_autorun_macro_phase = 6;
                    start_time = double(double(ros::Time::now().sec) + (double(ros::Time::now().nsec) / 1e9));
                }
                break;
            case 6: {
                double now_time = double(double(ros::Time::now().sec) + (double(ros::Time::now().nsec) / 1e9));
                drive_inverse_kinematics.linear.y = 0.2;
                if ((now_time - start_time) > 1) {
                    drive_inverse_kinematics.linear.y = 0;
                    cable_manager_autorun_macro_phase = 7;
                }
            } break;
            case 7:
                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::FOLLOW;
                C_cable_manager_controller.reference[ARM_MOTOR] = REFERENCE_CABLE_ARM_MOTOR - 35;

                if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::FOLLOW &&
                    F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR) == C_cable_manager_controller.reference[ARM_MOTOR]) {
                    cable_manager_autorun_macro_phase = 8;
                    start_time = double(double(ros::Time::now().sec) + (double(ros::Time::now().nsec) / 1e9));
                }
                break;
            case 8: {
                double now_time = double(double(ros::Time::now().sec) + (double(ros::Time::now().nsec) / 1e9));
                drive_inverse_kinematics.linear.y = 0.25;
                if ((now_time - start_time) > 2) {
                    drive_inverse_kinematics.linear.y = 0;
                    cable_manager_autorun_macro_phase = 10;
                }
            } break;

            case 10:
                ROS_INFO("F_cable_manager_controller.is_ended.at(ARM_MOTOR):%d", F_cable_manager_controller.is_ended.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR):%d", F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR));
                ROS_INFO("F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR):%lf", F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR));
                ROS_INFO("  ");
                C_cable_manager_controller.mode[ARM_MOTOR] = CONTROL_MODE::FOLLOW;
                C_cable_manager_controller.reference[ARM_MOTOR] = 0;

                if (F_cable_manager_controller.is_ended.at(ARM_MOTOR) &&
                    F_cable_manager_controller.mode_feedbackside.at(ARM_MOTOR) == CONTROL_MODE::FOLLOW &&
                    F_cable_manager_controller.reference_feedbackside.at(ARM_MOTOR) == C_cable_manager_controller.reference[ARM_MOTOR]) {
                    cable_manager_autorun_macro_phase = 0;
                }
                break;
        }
        if (cable_manager_push_step_number > 4) {
            cable_manager_push_step_number = 0;
        }
*/
