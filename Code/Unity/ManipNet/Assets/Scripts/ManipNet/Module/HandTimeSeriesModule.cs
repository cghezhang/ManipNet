#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class HandTimeSeriesModule : Module {

	public int PastKeys = 30;
	public int FutureKeys = 30;
	public float PastWindow = 1f;
	public float FutureWindow = 1f;
	public int Resolution = 1;

	// for smoothing / filtering
	public int filterSize = 0;
	public float filterPower = 1;

	public override ID GetID() {
		return ID.HandTimeSeries;
	}
	public override Module Initialise(MotionData data) {
		Data = data;
		return this;
	}
	public override void Slice(Sequence sequence) {}
	public override void Callback(MotionEditor editor) {}


	protected override void DerivedDraw(MotionEditor editor) {
		HandTimeSeries handTimeSeries = GetHandTimeSeries(editor.GetCurrentFrame(), editor.Mirror, 1f/editor.TargetFramerate, filterSize, filterPower);
		foreach(HandTimeSeries.Series series in handTimeSeries.Data) {
			if(series is HandTimeSeries.TransformationSeries && Data.GetModule(ID.HandObjectTransformation).Visualise) {
				HandObjectTransformationModule handTransformationModule = (HandObjectTransformationModule)Data.GetModule(ID.HandObjectTransformation);
				HandTimeSeries.TransformationSeries transformationSeries = (HandTimeSeries.TransformationSeries)series;
				if(handTransformationModule.drawLeftWristSeries){
					transformationSeries.Draw(transformationSeries.handTransformationSeries.leftWristTrans,
											  transformationSeries.handTransformationSeries.leftWristVel);
				}
				if(handTransformationModule.drawRightWristSeries){
					transformationSeries.Draw(transformationSeries.handTransformationSeries.rightWristTrans,
											  transformationSeries.handTransformationSeries.rightWristVel);
				}
				if(handTransformationModule.drawObjectSeries){
					int nObjects = transformationSeries.nObjects;
					for(int iObj=0; iObj<nObjects; iObj++){
						transformationSeries.Draw(transformationSeries.objectTransformationSeries[iObj].ObjectTrans, 
												  transformationSeries.objectTransformationSeries[iObj].ObjectAxisAngleV_R);
					}
				}
				// {
				// 	// draw axis angel 
				// 	UltiDraw.Begin();
				// 	Vector3 axisAngle = transformationSeries.objectTransformationSeries[0].ObjectAxisAngleV_R[10].GetRelativeDirectionFrom(transformationSeries.handTransformationSeries.rightWristTrans[10]);
				// 	Vector3 position = transformationSeries.objectTransformationSeries[0].ObjectTrans[10].GetPosition();
				// 	UltiDraw.DrawLine(position, position + axisAngle, 0.025f, 0f, UltiDraw.DarkGreen.Transparent(0.25f));
				// 	UltiDraw.End();
				// }
			}
			// if(series is HandTimeSeries.PoseSeries){
			// 	HandObjectTransformationModule handTransformationModule = (HandObjectTransformationModule)Data.GetModule(ID.HandObjectTransformation);
			// 	HandTimeSeries.PoseSeries poseSeries = (HandTimeSeries.PoseSeries)series;
			// 	if(handTransformationModule.drawPoseSeries){
			// 		poseSeries.Draw();
			// 	}
			// }

		}
	}

	protected override void DerivedInspector(MotionEditor editor) {
		PastKeys = EditorGUILayout.IntField("Past Keys", PastKeys);
		FutureKeys = EditorGUILayout.IntField("Future Keys", FutureKeys);
		PastWindow = EditorGUILayout.FloatField("Past Window", PastWindow);
		FutureWindow = EditorGUILayout.FloatField("Future Window", FutureWindow);
		Resolution = Mathf.Max(EditorGUILayout.IntField("Resolution", Resolution), 1);

		// for smoothing
		filterSize = Mathf.Max(EditorGUILayout.IntField("FilterSize",  filterSize), 0);
		filterPower = EditorGUILayout.FloatField("FilterPower", filterPower);
	}


	public HandTimeSeries GetHandTimeSeries(Frame frame, bool mirrored, float delta, int filterSize, float filterPower) {
		HandTimeSeries handTimeSeries_Origin = GetHandTimeSeries(frame, mirrored, PastKeys, FutureKeys, PastWindow, FutureWindow, Resolution, delta);
		// if filterSize = 0, then do not need to filter
		if(filterSize == 0){ 
			return handTimeSeries_Origin;
		}

		// other wise apply the filter/smooth below
		if(filterSize%2 == 0){ // if not an odd number, then +1
			filterSize += 1;
		}
		int addKeys = filterSize/2;
		float pastAddWindow = (float)(PastWindow) / (float)(PastKeys) * (float)addKeys;
		float futureAddWindow = (float)(FutureWindow) / (float)(FutureKeys) * (float)addKeys;
		// get longer handTimeSeries
		HandTimeSeries handTimeSeries_Long = GetHandTimeSeries(frame, mirrored, PastKeys+addKeys, FutureKeys+addKeys, 
														 	   PastWindow+pastAddWindow, FutureWindow+futureAddWindow, Resolution, delta);
		// filter here 
		HandTimeSeries.TransformationSeries transformationSeries_Long = (HandTimeSeries.TransformationSeries)handTimeSeries_Long.GetSeries(HandTimeSeries.Series.ID.TransformationSeries.ToString()); 
		int nObject = frame.numObjects;
		if(transformationSeries_Long != null){
			transformationSeries_Long.handTransformationSeries.leftWristTrans.GaussianFilter(filterSize, filterPower);
			transformationSeries_Long.handTransformationSeries.rightWristTrans.GaussianFilter(filterSize, filterPower);
			for(int iObj=0; iObj<nObject; iObj++){
				transformationSeries_Long.objectTransformationSeries[iObj].ObjectTrans.GaussianFilter(filterSize, filterPower);
				transformationSeries_Long.objectTransformationSeries[iObj].ObjectAxisAngleV_L.GaussianFilter(filterSize, filterPower);
				transformationSeries_Long.objectTransformationSeries[iObj].ObjectAxisAngleV_R.GaussianFilter(filterSize, filterPower);
			}
			// assign filtered to origin
			HandTimeSeries.TransformationSeries transformationSeries_Origin = (HandTimeSeries.TransformationSeries)handTimeSeries_Origin.GetSeries(HandTimeSeries.Series.ID.TransformationSeries.ToString()); 
			for(int i=0; i<handTimeSeries_Origin.Samples.Length; i++) {
				transformationSeries_Origin.handTransformationSeries.leftWristTrans[i] = transformationSeries_Long.handTransformationSeries.leftWristTrans[i+addKeys];
				transformationSeries_Origin.handTransformationSeries.rightWristTrans[i] = transformationSeries_Long.handTransformationSeries.rightWristTrans[i+addKeys];
				for(int iObj=0; iObj<nObject; iObj++){
					transformationSeries_Origin.objectTransformationSeries[iObj].ObjectTrans[i] = transformationSeries_Long.objectTransformationSeries[iObj].ObjectTrans[i+addKeys];
					transformationSeries_Origin.objectTransformationSeries[iObj].ObjectAxisAngleV_L[i] = transformationSeries_Long.objectTransformationSeries[iObj].ObjectAxisAngleV_L[i+addKeys];
					transformationSeries_Origin.objectTransformationSeries[iObj].ObjectAxisAngleV_R[i] = transformationSeries_Long.objectTransformationSeries[iObj].ObjectAxisAngleV_R[i+addKeys];
				}
			}
		}
		return handTimeSeries_Origin;
	}


	public HandTimeSeries GetHandTimeSeries(Frame frame, bool mirrored, int pastKeys, int futureKeys, float pastWindow, float futureWindow, int resolution, float delta) {
		HandTimeSeries handTimeSeries = new HandTimeSeries(pastKeys, futureKeys, pastWindow, futureWindow, resolution);
		int nObject = frame.numObjects;

		foreach(Module module in Data.Modules) {
			if(module is HandObjectTransformationModule) {
				{
					HandObjectTransformationModule m = (HandObjectTransformationModule)module;
					HandTimeSeries.TransformationSeries series = new HandTimeSeries.TransformationSeries(handTimeSeries, nObject);
					for(int i=0; i<handTimeSeries.Samples.Length; i++) {
						float t = frame.Timestamp + handTimeSeries.Samples[i].Timestamp;
						t = t<0f? 0f:t;
						t = t > Data.GetTotalTime()? Data.GetTotalTime():t;
						// series.handTransformationSeries.leftWristTrans[i] = m.GetLeftWristTransformation(Data.GetFrame(t), mirrored);
						// series.handTransformationSeries.rightWristTrans[i] = m.GetRightWristTransformation(Data.GetFrame(t), mirrored);	
						// series.handTransformationSeries.leftWristVel[i] = m.GetLeftWristVelocity(Data.GetFrame(t), mirrored, delta);
						// series.handTransformationSeries.rightWristVel[i] = m.GetRightWristVelocity(Data.GetFrame(t), mirrored, delta);
						// float[] leftWristAngularV = m.GetLeftWristAngularVelocity(Data.GetFrame(t), mirrored, delta);
						// float[] rightWristAngularV = m.GetRightWristAngularVelocity(Data.GetFrame(t), mirrored, delta);
						series.handTransformationSeries.leftWristTrans[i] = m.GetWristVisualTransformation(Data.GetFrame(t), 0, mirrored);
						series.handTransformationSeries.rightWristTrans[i] = m.GetWristVisualTransformation(Data.GetFrame(t), 1, mirrored);	
						series.handTransformationSeries.leftWristVel[i] = m.GetWristVisualVelocity(Data.GetFrame(t), 0, mirrored, delta);
						series.handTransformationSeries.rightWristVel[i] = m.GetWristVisualVelocity(Data.GetFrame(t), 1, mirrored, delta);
						float[] leftWristAngularV = m.GetWristVisualAngularVelocity(Data.GetFrame(t), 0, mirrored, delta);
						float[] rightWristAngularV = m.GetWristVisualAngularVelocity(Data.GetFrame(t), 1, mirrored, delta);
						for(int iAngle=0; iAngle<Frame.dimAngularVelocity; iAngle++){
							series.handTransformationSeries.leftWristAngularVel[i*Frame.dimAngularVelocity+iAngle] = leftWristAngularV[iAngle];
							series.handTransformationSeries.rightWristAngularVel[i*Frame.dimAngularVelocity+iAngle] = rightWristAngularV[iAngle];
						}
						for(int iObj=0; iObj<nObject; iObj++){
							series.objectTransformationSeries[iObj].ObjectTrans[i] = m.GetObjectTransformation(Data.GetFrame(t),iObj, mirrored);
							series.objectTransformationSeries[iObj].ObjectOriginalTrans[i] = m.GetOriginalObjecTransformation(Data.GetFrame(t),iObj, mirrored);
							series.objectTransformationSeries[iObj].ObjectAxisAngleV_L[i] = HandObjectTransformationModule.GetObjectVisualAxisAngleVelocity(Data.GetFrame(t),iObj,0, mirrored, delta);
							series.objectTransformationSeries[iObj].ObjectAxisAngleV_R[i] = HandObjectTransformationModule.GetObjectVisualAxisAngleVelocity(Data.GetFrame(t),iObj,1, mirrored, delta);
							
							series.objectTransformationSeries[iObj].ObjectRelativeTransV_L[i] = HandObjectTransformationModule.GetObjectVisualRelativeTransV(Data.GetFrame(t),iObj,0, mirrored, delta);
							series.objectTransformationSeries[iObj].ObjectRelativeTransV_R[i] = HandObjectTransformationModule.GetObjectVisualRelativeTransV(Data.GetFrame(t),iObj,1, mirrored, delta);
						}
					}
				}
							
				// {
				// 	// hard code here to add the poseSeries here in 
				// 	HandTimeSeries.PoseSeries series = new HandTimeSeries.PoseSeries(handTimeSeries);
				// 	for(int i=0; i<handTimeSeries.Samples.Length; i++) {
				// 		float t = frame.Timestamp + handTimeSeries.Samples[i].Timestamp;
				// 		t = t<0f? 0f:t;
				// 		t = t > Data.GetTotalTime()? Data.GetTotalTime():t;

				// 		Frame currentFrame = Data.GetFrame(t);
				// 		int mainIndex = mirrored? 0:1;
				// 		int viceIndex = mirrored? 1:0;
				// 		series.visualLeftPoses[i] = currentFrame.GetHandTransformations(viceIndex, mirrored);
				// 		series.visualRightPoses[i] = currentFrame.GetHandTransformations(mainIndex, mirrored);
				// 		series.visualLeftTipPositions[i] = currentFrame.GetHandTipPositions(viceIndex, mirrored);
				// 		series.visualRightTipPositions[i] = currentFrame.GetHandTipPositions(mainIndex, mirrored);
				// 		series.visualLeftVelocities[i] = currentFrame.GetHandVelocities(viceIndex, mirrored, delta);
				// 		series.visualRightVelocities[i] = currentFrame.GetHandVelocities(mainIndex, mirrored, delta);
				// 		series.visualLeftTipVelocities[i] = currentFrame.GetHandTipVelocities(viceIndex, mirrored, delta);
				// 		series.visualRightTipVelocities[i] = currentFrame.GetHandTipVelocities(mainIndex, mirrored, delta);

				// 	}
				// }

			}
			// if(module is HandActionModule) {
			// 	HandActionModule m = (HandActionModule)module;
			// 	HandTimeSeries.HandActionSeries series = new HandTimeSeries.HandActionSeries(handTimeSeries, m.GetNames());
			// 	for(int i=0; i<handTimeSeries.Samples.Length; i++) {
			// 		float t = frame.Timestamp + handTimeSeries.Samples[i].Timestamp;		
			// 		series.values[i] = m.GetActions(Data.GetFrame(t));
			// 		if(m.autoLabel.leftClosestDis.Length==m.Data.Frames.Length && m.autoLabel.rightClosestDis.Length==m.Data.Frames.Length){
			// 			series.contactValue[i] = m.autoLabel.GetActions(Data.GetFrame(t).Index-1, m.autoLabel.distanceThreshold, mirrored);
			// 		}
			// 	}
			// }
			// if(module is StyleModule) {
			// 	StyleModule m = (StyleModule)module;
			// 	HandTimeSeries.StyleSeries series = new HandTimeSeries.StyleSeries(handTimeSeries, m.GetNames());
			// 	for(int i=0; i<handTimeSeries.Samples.Length; i++) {
			// 		float t = frame.Timestamp + handTimeSeries.Samples[i].Timestamp;		
			// 		series.values[i] = m.GetStyles(Data.GetFrame(t));
			// 	}
			// }
			// if(module is HandContactModule){
			// 	HandContactModule m = (HandContactModule)module;
			// 	if(m.Baked){
			// 		// only if the contacts are baked
			// 		HandTimeSeries.ContactSeries series = new HandTimeSeries.ContactSeries(handTimeSeries);
			// 		for(int i=0; i<handTimeSeries.Samples.Length; i++) {
			// 			float t = frame.Timestamp + handTimeSeries.Samples[i].Timestamp;
			// 			t = t<0f? 0f:t;
			// 			t = t > Data.GetTotalTime()? Data.GetTotalTime():t;
			// 			// need to initialize hand contact module here
			// 			series.leftContactInfo[i] = m.GetVisualContactInfo(0, Data.GetFrame(t), mirrored);
			// 			series.rightContactInfo[i] = m.GetVisualContactInfo(1, Data.GetFrame(t), mirrored);
			// 		}
			// 	}
			// }
			
			// if(module is AlignmentModule) {
			//     AlignmentModule m = (AlignmentModule)module;
			//     HandTimeSeries.Alignment series = new HandTimeSeries.Alignment(handTimeSeries, m.GetIdentifiers());
			//     for(int i=0; i<handTimeSeries.Samples.Length; i++) {
			//         float t = frame.Timestamp + handTimeSeries.Samples[i].Timestamp;
			//         series.Phases[i] = m.GetPhases(t, mirrored);
			//         series.Magnitudes[i] = m.GetAmplitudes(t, mirrored);
			//     }
			// }
		}

		return handTimeSeries;
	}

}
#endif
