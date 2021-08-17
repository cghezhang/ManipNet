#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.IO;

public class HandObjectTransformationModule : Module {
// This is for calculating Hand and Object transformations

    public bool drawLeftWristSeries = false;
    public bool drawRightWristSeries = false;
    public bool drawObjectSeries = false;
    public bool applyRootOffset = true;

    // draw past pose
    public bool drawPoseSeries = false;

    // export the trajectory TXT
    public string trajectoryFolderPath = "";
	public bool selectedExport = false;
    public int selectedStart = 0;
    public int selectedEnd = 0;

    public override ID GetID() {
		return ID.HandObjectTransformation;
	}

	public override Module Initialise(MotionData data) {
		Data = data;
		return this;
	}

	public override void Slice(Sequence sequence) {}

	public override void Callback(MotionEditor editor) {}

	protected override void DerivedDraw(MotionEditor editor) {}

	protected override void DerivedInspector(MotionEditor editor) {

        EditorGUILayout.BeginHorizontal();
        if(Utility.GUIButton("DrawLeftWristSeries", drawLeftWristSeries ? UltiDraw.Cyan : UltiDraw.LightGrey, UltiDraw.Black, 150f, 20f)) {
				drawLeftWristSeries = !drawLeftWristSeries;
		}
        if(Utility.GUIButton("DrawRightWristSeries", drawRightWristSeries ? UltiDraw.Cyan : UltiDraw.LightGrey, UltiDraw.Black, 150f, 20f)) {
				drawRightWristSeries = !drawRightWristSeries;
		}
        if(Utility.GUIButton("DrawObjectSeries", drawObjectSeries ? UltiDraw.Cyan : UltiDraw.LightGrey, UltiDraw.Black, 150f, 20f)) {
				drawObjectSeries = !drawObjectSeries;
		}
        if(Utility.GUIButton("DrawPoseSeries", drawPoseSeries ? UltiDraw.Cyan : UltiDraw.LightGrey, UltiDraw.Black, 150f, 20f)) {
				drawPoseSeries = !drawPoseSeries;
		}
        EditorGUILayout.EndHorizontal();
        if(Utility.GUIButton("Apply Object Root Offset", applyRootOffset ? UltiDraw.Cyan : UltiDraw.LightGrey, UltiDraw.Black, 150f, 20f)) {
				applyRootOffset = !applyRootOffset;
		}

        // export the trajectory TXT
        using(new EditorGUILayout.VerticalScope ("Box")) {
            trajectoryFolderPath = EditorGUILayout.TextField(trajectoryFolderPath); 
            if(Utility.GUIButton("Export Trajectory TXT", UltiDraw.LightGrey, UltiDraw.Black, 150f, 20f)) {
                ExportTrajectoryTXT();
                
		    }
            if(Utility.GUIButton("Selected Export", selectedExport ? UltiDraw.Cyan : UltiDraw.LightGrey, UltiDraw.Black, 150f, 20f)) {
				selectedExport = !selectedExport;
		    }
            if(selectedExport){
                selectedStart = EditorGUILayout.IntField("Start Frame:", selectedStart);
                selectedEnd = EditorGUILayout.IntField("End Frame:", selectedEnd);
            }
        }        
    }
    // export the trajectory TXT
    public void ExportTrajectoryTXT(){
        /// <summary>
        /// write the object leftHand rightHand, QX QY QZ QW PX PY PZ
        /// hard code for only one object
        /// <summary>     
        Debug.Log(trajectoryFolderPath);
        if(Directory.Exists(trajectoryFolderPath)){
            string fileName = trajectoryFolderPath + Data.name + ".txt";
            Debug.Log(fileName);
            StreamWriter streamWriter = new StreamWriter(fileName);
            // write the framerate
            streamWriter.WriteLine(Data.Framerate.ToString());
            // write the object leftHand rightHand, QX QY QZ QW PX PY PZ
            Matrix4x4 centerLocal = Data.GetSceneObject(0).GetComponent<ObjectManager>().GetCubeLocalTransformation(Axis.None);
            int exportStart = 0;
            int exportEnd = Data.Frames.Length;
            if(selectedExport && selectedStart<selectedEnd && selectedEnd<exportEnd && selectedStart>exportStart){
                Debug.Log("only export from " + selectedStart + " to " + selectedEnd);
                exportStart = selectedStart;
                exportEnd = selectedEnd;
            }
            for(int iFrame=exportStart; iFrame<exportEnd;iFrame++){
                List<float> value = new List<float>();
                {
                    Matrix4x4 objectTrans = Data.Frames[iFrame].GetObjectTransformation(0, false) * centerLocal;
                    Vector3 position  = objectTrans.GetPosition();
                    Quaternion rotation = objectTrans.GetRotation();
                    value.Add(rotation.x);
                    value.Add(rotation.y);
                    value.Add(rotation.z);
                    value.Add(rotation.w);
                    value.Add(position.x);
                    value.Add(position.y);
                    value.Add(position.z);
                }
                {
                    Matrix4x4 leftTrans = Data.Frames[iFrame].GetHandWristTransformation(0, false);
                    Vector3 position  = leftTrans.GetPosition();
                    Quaternion rotation = leftTrans.GetRotation();
                    value.Add(rotation.x);
                    value.Add(rotation.y);
                    value.Add(rotation.z);
                    value.Add(rotation.w);
                    value.Add(position.x);
                    value.Add(position.y);
                    value.Add(position.z);
                }
                {
                    Matrix4x4 rightTrans = Data.Frames[iFrame].GetHandWristTransformation(1, false);
                    Vector3 position  = rightTrans.GetPosition();
                    Quaternion rotation = rightTrans.GetRotation();
                    value.Add(rotation.x);
                    value.Add(rotation.y);
                    value.Add(rotation.z);
                    value.Add(rotation.w);
                    value.Add(position.x);
                    value.Add(position.y);
                    value.Add(position.z);
                }
                streamWriter.WriteLine(string.Join(",", value));
            }
            streamWriter.Close();
        }
    }

	public void DetectSetup() {}



    // in Frame class, left is real left, where we use 0 for left hand and 1 for right hand.
    // but here, we get left and right from visualization
    public Matrix4x4 GetWristVisualTransformation(Frame frame, int iHand, bool mirrored) {
        if(iHand == 0){
            if(mirrored){
                // right hand
                return frame.GetHandWristTransformation(iHand+1, mirrored);
            }
            else{
                return frame.GetHandWristTransformation(iHand, mirrored);
            }
        }
        if(iHand == 1){
            if(mirrored){
                // left hand
                return frame.GetHandWristTransformation(iHand-1, mirrored);
            }
            else{
                return frame.GetHandWristTransformation(iHand, mirrored);
            }
        }
        Debug.LogError("hand index " + iHand +  " is wrong");
        return Matrix4x4.identity;
    }
    public Vector3 GetWristVisualVelocity(Frame frame, int iHand, bool mirrored, float delta) {
        if(iHand == 0){
            if(mirrored){
                // right hand
                return frame.GetWristVelocity(iHand+1, mirrored, delta);
            }
            else{
                return frame.GetWristVelocity(iHand, mirrored, delta);
            }
        }
        if(iHand == 1){
            if(mirrored){
                // left hand
                return frame.GetWristVelocity(iHand-1, mirrored, delta);
            }
            else{
                return frame.GetWristVelocity(iHand, mirrored, delta);
            }
        }
        Debug.LogError("hand index " + iHand +  " is wrong");
        return Vector3.zero;
    }

    public float[] GetWristVisualAngularVelocity(Frame frame, int iHand, bool mirrored, float delta) {
        if(iHand == 0){
            if(mirrored){
                return frame.GetWristAngularVelocity(iHand+1, mirrored, delta);
            }
            else{
                return frame.GetWristAngularVelocity(iHand, mirrored, delta);
            }
        }
        if(iHand == 1){
            if(mirrored){
                return frame.GetWristAngularVelocity(iHand-1, mirrored, delta);
            }
            else{
                return frame.GetWristAngularVelocity(iHand, mirrored, delta);
            }
        }
        Debug.LogError("hand index " + iHand +  " is wrong");
        return new float[Frame.dimAngularVelocity];
    }

    public Matrix4x4 GetObjectTransformation(Frame frame, int indexObj, bool mirrored) {
        // return original transformation if not apply the RootOffset, otherwise return center
        Matrix4x4 m = frame.GetObjectTransformation(indexObj, mirrored);
        
        if(applyRootOffset){
            Axis mirrorAxis = mirrored? frame.Data.MirrorAxis:Axis.None;
            Matrix4x4 local = Data.GetSceneObject(indexObj).GetComponent<ObjectManager>().GetCubeLocalTransformation(mirrorAxis);
            m = local.GetRelativeTransformationFrom(m);
        }
		return m;
	}

    // redundant, will remove later, it is just designed for getting future transformation
    public Matrix4x4 GetOriginalObjecTransformation(Frame frame, int indexObj, bool mirrored) {
        return frame.GetObjectTransformation(indexObj, mirrored);
	}

    public static Matrix4x4 GetObjectVisualRelativeTransV(Frame frame, int indexObj, int indexHand, bool mirrored, float delta) {
        int m = mirrored? 1:0;
        return frame.GetObjectRelativeTransV(indexObj, Math.Abs(indexHand-m), mirrored, delta);
    } 
    public static Vector3 GetObjectVisualAxisAngleVelocity(Frame frame, int indexObj, int indexHand, bool mirrored, float delta) {
        int m = mirrored? 1:0;
        return frame.GetObjectRelativeAxisAngleV(indexObj, Math.Abs(indexHand-m), mirrored, delta);
    } 
}
#endif
