#include "state_machine.hpp"
#include "error_handle.hpp"

void StateMachine::HandleEvent(enum Event event) {
    switch (main_state_) {

        case kMainStateNone:  //初始状态

            switch (event) {
                case kEventEnterOperate:  //切换到运行
                    main_state_ = kOperate;
                    break;

                case kEventEnterHalt:  //切换到停止
                    main_state_ = kHalt;
                    break;

                case kEventSwitchSubMode00:  //切换到子状态00
                    sub_state_ = kSubMode00;
                    break;

                case kEventSwitchSubMode11:  //切换到子状态11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //切换到子状态12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //切换到子状态13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //切换到子状态21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //切换到子状态22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //切换到子状态23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //切换到子状态31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //切换到子状态32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //切换到子状态33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    /*ErrorHandle(kSwitchError);*/
                    break;
            }

            break;

        case kHalt:  //停滞状态

            switch (event) {
                case kEventEnterOperate:  //切换到运行
                    main_state_ = kOperate;
                    break;

                case kEventEnterHalt:  //切换到停止
                    main_state_ = kHalt;
                    break;

                case kEventSwitchSubMode00:  //切换到子状态00
                    sub_state_ = kSubMode00;
                    break;

                case kEventSwitchSubMode11:  //切换到子状态11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //切换到子状态12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //切换到子状态13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //切换到子状态21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //切换到子状态22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //切换到子状态23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //切换到子状态31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //切换到子状态32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //切换到子状态33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    /*ErrorHandle(kSwitchError);*/
                    break;
            }

            break;

        case kOperate:  //运行状态

            switch (event) {
                case kEventEnterOperate:  //切换到运行
                    main_state_ = kOperate;
                    break;

                case kEventEnterHalt:  //切换到停止
                    main_state_ = kHalt;
                    sub_state_ = kSubStateNone;  //子状态也被重置
                    break;

                case kEventSwitchSubMode00:  //切换到子状态00
                    sub_state_ = kSubMode00;
                    break;

                case kEventSwitchSubMode11:  //切换到子状态11
                    sub_state_ = kSubMode11;
                    break;

                case kEventSwitchSubMode12:  //切换到子状态12
                    sub_state_ = kSubMode12;
                    break;

                case kEventSwitchSubMode13:  //切换到子状态13
                    sub_state_ = kSubMode13;
                    break;

                case kEventSwitchSubMode21:  //切换到子状态21
                    sub_state_ = kSubMode21;
                    break;

                case kEventSwitchSubMode22:  //切换到子状态22
                    sub_state_ = kSubMode22;
                    break;

                case kEventSwitchSubMode23:  //切换到子状态23
                    sub_state_ = kSubMode23;
                    break;

                case kEventSwitchSubMode31:  //切换到子状态31
                    sub_state_ = kSubMode31;
                    break;

                case kEventSwitchSubMode32:  //切换到子状态32
                    sub_state_ = kSubMode32;
                    break;

                case kEventSwitchSubMode33:  //切换到子状态33
                    sub_state_ = kSubMode33;
                    break;

                default:
                    break;
            }

            break;

        default:
            break;
    }
}
