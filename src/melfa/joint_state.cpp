#include <iostream>

#include "melfa/joint_state.h"

std::ostream& operator<< (std::ostream& ostr, const melfa::JointState& joint_state)
{
    ostr << "(" 
         << "j1: " << joint_state.j1 << ", "
         << "j2: " << joint_state.j2 << ", "
         << "j3: " << joint_state.j3 << ", "
         << "j4: " << joint_state.j4 << ", "
         << "j5: " << joint_state.j5 << ", "
         << "j6: " << joint_state.j6 << ")";
    return ostr;
}

