#ifndef JOINT_STATE_H_
#define JOINT_STATE_H_

namespace melfa
{
    /**
    * \brief struct to hold a joint state
    * The angles are given in radiants.
    */
    struct JointState
    {
        double j1;
        double j2;
        double j3;
        double j4;
        double j5;
        double j6;
    };
}

std::ostream& operator<< (std::ostream& ostr, const melfa::JointState& joint_state);

#endif

