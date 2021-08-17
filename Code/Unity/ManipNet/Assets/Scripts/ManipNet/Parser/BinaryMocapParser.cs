using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;

public class BinaryMocapParser
{
    // constant
    public const Int32 N_HANDS = 2;
    public const Int32 NUMBER_OF_FRAMES = 17; // it is indeed the number of bones
    public const UInt32 legacyNumJointAngles = 16;
    public const Int32 NUMBER_OF_JOINTS = 22; 
    public const int NUMBER_OF_DOFS_WRIST = 2;
}