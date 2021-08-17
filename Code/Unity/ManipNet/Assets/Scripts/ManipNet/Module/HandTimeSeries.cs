using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTimeSeries {

    public Sample[] Samples = new Sample[0];
	public Series[] Data = new Series[0];
    public int Pivot = 0;
    public int Resolution = 0;
	public float PastWindow = 0f;
	public float FutureWindow = 0f;

	public int PastSampleCount {
		get {return Pivot;}
	}
	public int FutureSampleCount {
		get {return Samples.Length-Pivot-1;}
	}
	public int KeyCount {
		get {return PastKeyCount + FutureKeyCount + 1;}
	}
	public int PivotKey {
		get {return Pivot / Resolution;}
	}
	public int PastKeyCount {
		get {return Pivot / Resolution;}
	}
	public int FutureKeyCount {
		get {return (Samples.Length-Pivot-1) / Resolution;}
	}

    public HandTimeSeries(int pastKeys, int futureKeys, float pastWindow, float futureWindow, int resolution) {
        int samples = pastKeys + futureKeys + 1;
        if(samples == 1 && resolution != 1) {
            resolution = 1;
            Debug.Log("Resolution corrected to 1 because only one sample is available.");
        }

        Samples = new Sample[(samples-1)*resolution+1];
        Pivot = pastKeys*resolution;
        Resolution = resolution;

        for(int i=0; i<Pivot; i++) {
            Samples[i] = new Sample(i, -pastWindow + i*pastWindow/(pastKeys*resolution));
        }
        Samples[Pivot] = new Sample(Pivot, 0f);
        for(int i=Pivot+1; i<Samples.Length; i++) {
            Samples[i] = new Sample(i, (i-Pivot)*futureWindow/(futureKeys*resolution));
        }

		PastWindow = pastWindow;
		FutureWindow = futureWindow;
    }
	
	private void Add(Series series) {
		ArrayExtensions.Add(ref Data, series);
		if(series.HandTimeSeries != null) {
			Debug.Log("Data is already added to another time series.");
		} else {
			series.HandTimeSeries = this;
		}
	}

	public Series GetSeries(string type) {
		for(int i=0; i<Data.Length; i++) {
			if(Data[i].GetID().ToString() == type) {
				return Data[i];
			}
		}
		Debug.Log("Series of type " + type + " could not be found.");
		return null;
		//return System.Array.Find(Data, x => x.GetID().ToString() == type);
	}
	


    public Sample GetPivot() {
        return Samples[Pivot];
    }

    public Sample GetKey(int index) {
		if(index < 0 || index >= KeyCount) {
			Debug.Log("Given key was " + index + " but must be within 0 and " + (KeyCount-1) + ".");
			return null;
		}
        return Samples[index*Resolution];
    }

	public Sample GetPreviousKey(int sample) {
		if(sample < 0 || sample >= Samples.Length) {
			Debug.Log("Given index was " + sample + " but must be within 0 and " + (Samples.Length-1) + ".");
			return null;
		}
		return GetKey(sample/Resolution);
	}

	public Sample GetNextKey(int sample) {
		if(sample < 0 || sample >= Samples.Length) {
			Debug.Log("Given index was " + sample + " but must be within 0 and " + (Samples.Length-1) + ".");
			return null;
		}
		if(sample % Resolution == 0) {
			return GetKey(sample/Resolution);
		} else {
			return GetKey(sample/Resolution + 1);
		}
	}

    public class Sample {
        public int Index;
        public float Timestamp;

        public Sample(int index, float timestamp) {
		    Index = index;
            Timestamp = timestamp;
        }
    }

	public abstract class Series {
		public enum ID {TransformationSeries};
		public HandTimeSeries HandTimeSeries;
		public abstract ID GetID();
	}

	public class TransformationSeries : Series {
		public int nSamples;
		public int nObjects;
		public HandTransformationSeries handTransformationSeries;
		public ObjectTransformationSeries[] objectTransformationSeries;

		public override ID GetID() {
			return Series.ID.TransformationSeries;
		}

		public TransformationSeries(HandTimeSeries handTimeSeries, int nObjects_in) {
			nSamples = handTimeSeries.Samples.Length;
			nObjects = nObjects_in;
			handTimeSeries.Add(this);
			handTransformationSeries = new HandTransformationSeries(nSamples);
			objectTransformationSeries = new ObjectTransformationSeries[nObjects];
			for(int iObj=0; iObj<nObjects; iObj++){
				objectTransformationSeries[iObj] = new ObjectTransformationSeries(nSamples, iObj);
			}
		}

		public void Draw(Matrix4x4[] Transformations, Vector3[] Velocities) {
			int step = HandTimeSeries.Resolution;
			UltiDraw.Begin();
			//Connections
			for(int i=0; i<Transformations.Length-step; i+=step) {
				UltiDraw.DrawLine(Transformations[i].GetPosition(), Transformations[i+step].GetPosition(), 0.02f, UltiDraw.Black);
			}

			//Positions
			for(int i=0; i<Transformations.Length; i+=step) {
				UltiDraw.DrawCircle(Transformations[i].GetPosition(), 0.025f, UltiDraw.Black);
			}

			//Directions
			for(int i=0; i<Transformations.Length; i+=step) {
				UltiDraw.DrawLine(Transformations[i].GetPosition(), Transformations[i].GetPosition() + 0.1f*Transformations[i].GetForward(), 0.025f, 0f, UltiDraw.Blue.Transparent(0.5f));
			}
			for(int i=0; i<Transformations.Length; i+=step) {
				UltiDraw.DrawLine(Transformations[i].GetPosition(), Transformations[i].GetPosition() + 0.1f*Transformations[i].GetUp(), 0.025f, 0f, UltiDraw.Green.Transparent(0.5f));
			}
			for(int i=0; i<Transformations.Length; i+=step) {
				UltiDraw.DrawLine(Transformations[i].GetPosition(), Transformations[i].GetPosition() + 0.1f*Transformations[i].GetRight(), 0.025f, 0f, UltiDraw.Red.Transparent(0.5f));
			}

			// //Velocities
			// for(int i=0; i<Velocities.Length; i+=step) {
			// 	UltiDraw.DrawLine(Transformations[i].GetPosition(), Transformations[i].GetPosition() + Velocities[i], 0.025f, 0f, UltiDraw.DarkGreen.Transparent(0.25f));
			// }
			
			UltiDraw.End();
		}

		public class HandTransformationSeries{
			public Matrix4x4[] leftWristTrans;
			public Vector3[] leftWristVel;
			public float[] leftWristAngularVel;
			public Matrix4x4[] rightWristTrans;
			public Vector3[] rightWristVel;
			public float[] rightWristAngularVel;

			public HandTransformationSeries(int nSamples){
				leftWristTrans = new Matrix4x4[nSamples];
				leftWristVel = new Vector3[nSamples];
				leftWristAngularVel = new float[nSamples*6];
				rightWristTrans = new Matrix4x4[nSamples];
				rightWristVel = new Vector3[nSamples];
				rightWristAngularVel = new float[nSamples*6];
			}
		}
		
		public class ObjectTransformationSeries{
			public int indexObject;
			public Matrix4x4[] ObjectTrans; // here is for calculating object mesh center transformation, because it maybe not in the center
			public Matrix4x4[] ObjectOriginalTrans; // here is for calculating object original trans
			public Vector3[] ObjectAxisAngleV_L;
			public Vector3[] ObjectAxisAngleV_R;
			
			// transformation velocity, first relative to the wrist then calculate the relative transformation
			public Matrix4x4[] ObjectRelativeTransV_L;
			public Matrix4x4[] ObjectRelativeTransV_R;
			public ObjectTransformationSeries(int nSamples, int index){
				indexObject = index;
				ObjectTrans = new Matrix4x4[nSamples];
				ObjectOriginalTrans = new Matrix4x4[nSamples];
				ObjectAxisAngleV_L = new Vector3[nSamples];
				ObjectAxisAngleV_R = new Vector3[nSamples];
				ObjectRelativeTransV_L = new Matrix4x4[nSamples];
				ObjectRelativeTransV_R = new Matrix4x4[nSamples];
			}
		}
	}
}	