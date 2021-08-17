using UnityEngine;

public class CuboidMapModule  {

	public Vector3 Size = Vector3.one;
	public Vector3Int Resolution = new Vector3Int(10, 10, 10);
	public LayerMask Mask = -1;
	public Color ColorEmpty = UltiDraw.Black;
	public float transEmpty = 0.2f;
	public Color ColorOccupied = UltiDraw.Cyan;
	public float transOccupied = 0.2f;
	// public bool DrawReferences = false;
	// public bool DrawDistribution = false;
	// public UltiDraw.GUIRect Rect = new UltiDraw.GUIRect(0.5f, 0.1f, 0.9f, 0.1f);

	// for cuboidMap displacement, we say this is for right hand 
	public Vector3 cuboidOffset = Vector3.zero;

	/* 
	sort of hard code here, but could modify the cuboid transformation from here
	*/
	public Matrix4x4 GetCuboidTransformation(Frame frame, bool mirrored, int handIndex){
		Matrix4x4 cuboidT = Matrix4x4.identity;
		Matrix4x4 wristJointT = frame.GetHandTransformation(handIndex, 0, mirrored);
		Matrix4x4 middleJointT = frame.GetHandTransformation(handIndex, 8, mirrored);

		Vector3 cuboidP = middleJointT.GetPosition();
		Quaternion cuboidR = middleJointT.GetRotation();

		// offset here
		Vector3 offset = cuboidOffset;
		// first get the mirrored local offset (relative to the wrist) 
		if(handIndex==0 && !mirrored || handIndex==1 && mirrored){
			/// <summary>
			/// note here is for vice hand (0, unmirrored) and (1, mirrored)
			/// </summary>
			offset = offset.GetMirror(frame.Data.MirrorAxis);
		}
		// then get the current local position (relative to the wrist)
		cuboidP = cuboidP.GetRelativePositionTo(wristJointT);
		// then apply the offset and get the global position
		cuboidP += offset;
		cuboidP = cuboidP.GetRelativePositionFrom(wristJointT);
		// finally get the transformations
		cuboidT = Matrix4x4.TRS(cuboidP,cuboidR, Vector3.one);
		return cuboidT;
	}

	public static Matrix4x4 GetCuboidTransformationFromWrist(Actor hand, int handIndex, Axis mirrorAxis, Vector3 offset){
		Matrix4x4 cuboidT = Matrix4x4.identity;
		Matrix4x4 wristJointT = hand.Bones[0].Transform.GetWorldMatrix().GetMirror(mirrorAxis);
		Matrix4x4 middleJointT = hand.Bones[8].Transform.GetWorldMatrix().GetMirror(mirrorAxis);

		Vector3 cuboidP = middleJointT.GetPosition();
		Quaternion cuboidR = middleJointT.GetRotation();

		// first get the mirrored local offset (relative to the wrist) 
		if(handIndex==0 && mirrorAxis==Axis.None || handIndex==1 && mirrorAxis!=Axis.None){
			/// <summary>
			/// todo hard code Xpositive, because we build left hand from right by mirrroing along X Axis
			/// herenote here is for vice hand (0, unmirrored) and (1, mirrored)
			/// </summary>
			offset = offset.GetMirror(Axis.XPositive);
		}
		// then get the current local position (relative to the wrist)
		cuboidP = cuboidP.GetRelativePositionTo(wristJointT);
		// then apply the offset and get the global position
		cuboidP += offset;
		cuboidP = cuboidP.GetRelativePositionFrom(wristJointT);
		// finally get the transformations
		cuboidT = Matrix4x4.TRS(cuboidP,cuboidR, Vector3.one);
		return cuboidT;
	}
}
