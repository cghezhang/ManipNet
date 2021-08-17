using UnityEngine;

[System.Serializable]
public class CuboidMap {

	public Matrix4x4 Pivot = Matrix4x4.identity;
	public Vector3[] Points = new Vector3[0];

	public Vector3[] References = new Vector3[0];
	public float[] Occupancies = new float[0];
    public float[] Distance = new float[0];

    // here stores the voxel index and the local position
    public KDTree.KDTree<int> KDTree = null;
    public KDTree.KDTree<int> KDTree_mirror = null;

	public Vector3Int Resolution = new Vector3Int(10, 10, 10);
	public Vector3 Size = Vector3.one;

    public void ResetDistance(int length){
        Distance = new float[length];
        for(int i=0; i<length; i++){
            Distance[i] =999f;
        }
    }
    public void ResetDistance(){
        int length = Occupancies.Length;
        Distance = new float[length];
        for(int i=0; i<length; i++){
            Distance[i] =999f;
        }
    }

	public CuboidMap(Vector3Int resolution) {
		Size = Vector3.zero;
		Resolution = resolution;
		Generate();
	}

	public void Setup(Vector3Int resolution) {
		if(Resolution != resolution) {
			Resolution = resolution;
			Generate();
		}
	}

    public int GetOccupiedNumber(){
        int num = 0;
        for(int i=0; i<Occupancies.Length; i++){
            if(Occupancies[i]>0){
                num++;
            }
        }
        return num;
    }

    public Vector3 GetStep() {
        return new Vector3(Size.x / Resolution.x, Size.y / Resolution.y, Size.z / Resolution.z);
    }

    private int GetDimensionality() {
        return Resolution.x * Resolution.y * Resolution.z;
    }

	public void Generate() {
        Points = new Vector3[GetDimensionality()];
        References = new Vector3[GetDimensionality()];
        Occupancies = new float[GetDimensionality()];
        ResetDistance();
        for(int x=0; x<Resolution.x; x++) {
            for(int y=0; y<Resolution.y; y++) {
                for(int z=0; z<Resolution.z; z++) {
                    Points[x*Resolution.y*Resolution.z + y*Resolution.z + z] = new Vector3(
                        -0.5f + (x+0.5f)/Resolution.x,
                        -0.5f + (y+0.5f)/Resolution.y,
                        -0.5f + (z+0.5f)/Resolution.z
                    );
                }
            }
        }
	}

	public void Sense(Matrix4x4 pivot, LayerMask mask, Vector3 size, float smoothing=0f, bool extractBoundary = false) {
		Pivot = Utility.Interpolate(Pivot, pivot, 1f-smoothing);
        Size = smoothing*Size + (1f-smoothing)*size;

		Vector3 pivotPosition = Pivot.GetPosition();
		Quaternion pivotRotation = Pivot.GetRotation();
        Vector3 sensorPosition = pivot.GetPosition();
        Quaternion sensorRotation = pivot.GetRotation();
        Vector3 step = GetStep();
        float range = Mathf.Max(step.x, step.y, step.z);
        for(int i=0; i<Points.Length; i++) {
            if(Size == Vector3.zero) {
                References[i] = pivotPosition;
                Occupancies[i] = 0f;
            } else {
                References[i] = pivotPosition + pivotRotation * Vector3.Scale(Points[i], Size);
                Vector3 sensor = sensorPosition + sensorRotation * Vector3.Scale(Points[i], Size);
                Collider c;
                Vector3 closest = Utility.GetClosestPointOverlapBox(sensor, step/2f, sensorRotation, mask, out c);
                Occupancies[i] = smoothing*Occupancies[i] + (1f-smoothing)*(c == null ? 0f : 1f - Vector3.Distance(sensor, closest) / range);
            }
        }
        if(extractBoundary == true){
            ExtractBoundary();
        }
        // reset the distance after the update
        ResetDistance();
	}

    public void Retransform(Matrix4x4 pivot, Axis mirrorAxis) {
        /** 
        here need to add mirror axis because the points is relative to pivot and when pivot mirror, these thing need to mirror as well
        */
        Pivot = pivot;
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Pivot.GetRotation();
        for(int i=0; i<References.Length; i++) {
            References[i] = position + rotation * Vector3.Scale(Points[i].GetMirror(mirrorAxis), Size);
        }
    }

    // KD tree stores the voxel position relative to pivot, so later for KNN search the point need to be relative to pivot as well
    public void RebuildKDTree(Axis mirrorAxis){
        if(mirrorAxis == Axis.None){
            KDTree = new KDTree.KDTree<int>(3);
            for(int i=0; i<References.Length; i++) {
                Vector3 point = References[i].GetRelativePositionTo(Pivot);
                KDTree.AddPoint(new double[3]{point.x, point.y, point.z}, i);
            }
        }
        else{
            // if mirror need to rebuild a tree because the relative information changed/ local transformation of the voxel changes
            KDTree_mirror = new KDTree.KDTree<int>(3);
            for(int i=0; i<References.Length; i++) {
                // here is still the References which are already mirrored in the retransform function
                Vector3 point = References[i].GetRelativePositionTo(Pivot);
                KDTree_mirror.AddPoint(new double[3]{point.x, point.y, point.z}, i);
            }
        }
        
    }

    public void ExtractBoundary(){
        float[] newOccupancies = new float[Points.Length];
        for(int y=0; y<Resolution.y; y++) {
            for(int x=0; x<Resolution.x; x++) {
                if(Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + 0] == 1f){
                    newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + 0] = 1f;
                }
                if(Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + Resolution.z-1] == 1f){
                    newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + Resolution.z-1] = 1f;
                }
                for(int z=1; z<Resolution.z-1; z++) {
                    if(Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z-1] <1f && Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] ==1f){
                        // boundary 1, previous <1, current =1
                        newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] =1f;
                    }
                    else if(Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] ==1f && Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z+1]<1f){
                        // boundary 2, 
                        newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] =1f;
                    }
                    
                }
            }
        }

        for(int z=0; z<Resolution.z; z++) {
            for(int x=0; x<Resolution.x; x++) {
                if(Occupancies[0*Resolution.x*Resolution.z + x*Resolution.z + z] == 1f){
                    newOccupancies[0*Resolution.x*Resolution.z + x*Resolution.z + z] = 1f;
                }
                if(Occupancies[(Resolution.y-1)*Resolution.x*Resolution.z + x*Resolution.z + z] == 1f){
                    newOccupancies[(Resolution.y-1)*Resolution.x*Resolution.z + x*Resolution.z + z] =1f;
                } 
                for(int y=1; y<Resolution.y-1; y++) {                   
                    if(Occupancies[(y-1)*Resolution.x*Resolution.z + x*Resolution.z + z] <1f && Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z]==1f){
                        // boundary 1, previous <1, current =1
                        newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] =1f;
                    }
                    else if(Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] ==1f && Occupancies[(y+1)*Resolution.x*Resolution.z + x*Resolution.z + z]<1f){
                        // boundary 2, 
                        newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] =1f;
                    }
                }
            }
        }

        for(int z=0; z<Resolution.z; z++) {
            for(int y=0; y<Resolution.y; y++) {
                if(Occupancies[y*Resolution.x*Resolution.z + 0*Resolution.z + z] == 1f){
                        newOccupancies[y*Resolution.x*Resolution.z + 0*Resolution.z + z] = 1f;
                }
                if(Occupancies[y*Resolution.x*Resolution.z + (Resolution.x-1)*Resolution.z + z] == 1f){
                    newOccupancies[y*Resolution.x*Resolution.z + (Resolution.x-1)*Resolution.z + z] = 1f;
                }
                for(int x=1; x<Resolution.x-1; x++) {
                    if(Occupancies[y*Resolution.x*Resolution.z + (x-1)*Resolution.z + z] <1f && Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z]==1f){
                        // boundary 1, previous <1, current =1
                        newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] =1f;
                    }
                    else if(Occupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] ==1f && Occupancies[y*Resolution.x*Resolution.z + (x+1)*Resolution.z + z]<1f){
                        // boundary 2, 
                        newOccupancies[y*Resolution.x*Resolution.z + x*Resolution.z + z] =1f;
                    }
                }
            }
        }

        Occupancies = newOccupancies;
    }

	public void Draw(Color colorEmpty, Color colorOccupied) {
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Pivot.GetRotation();
        UltiDraw.Begin();
        Vector3 step = GetStep();
		if(Size != Vector3.zero) {
            UltiDraw.DrawWireCuboid(position, rotation, Size, colorEmpty);
            for(int i=0; i<Points.Length; i++) {
                if(Occupancies[i] == 0f) {
                    UltiDraw.DrawCuboid(References[i], rotation, step, colorEmpty);
                }
            }
            for(int i=0; i<Points.Length; i++) {
                if(Occupancies[i] > 0f) {
                    UltiDraw.DrawCuboid(References[i], rotation, step, Color.Lerp(colorEmpty, colorOccupied, Occupancies[i]));
                }
            }
		}

        // UltiDraw.DrawCuboid(References[0], rotation, step, UltiDraw.Red);
        // UltiDraw.DrawCuboid(References[Resolution.x*Resolution.y*Resolution.z-1], rotation, step, UltiDraw.Blue);
        // UltiDraw.DrawCuboid(References[Resolution.x*Resolution.y-1], rotation, step, UltiDraw.Green);
        UltiDraw.End();
	}

    public void Draw(Color colorOccupied) {
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Pivot.GetRotation();
        UltiDraw.Begin();
        Vector3 step = GetStep();
		if(Size != Vector3.zero) {
            UltiDraw.DrawWireCuboid(position, rotation, Size, colorOccupied);
            for(int i=0; i<Points.Length; i++) {
                if(Occupancies[i] > 0f) {
                    UltiDraw.DrawCuboid(References[i], rotation, step, Color.Lerp(UltiDraw.None, colorOccupied, Occupancies[i]));
                }
            }
		}

        // UltiDraw.DrawCuboid(References[0], rotation, step, UltiDraw.Red);
        // UltiDraw.DrawCuboid(References[Resolution.x*Resolution.y*Resolution.z-1], rotation, step, UltiDraw.Blue);
        // UltiDraw.DrawCuboid(References[Resolution.x*Resolution.y-1], rotation, step, UltiDraw.Green);
        UltiDraw.End();
	}

    public void DrawContact(Color free, Color colission, float threshold) {
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Pivot.GetRotation();
        UltiDraw.Begin();
        Vector3 step = GetStep();
		if(Size != Vector3.zero) {
            UltiDraw.DrawWireCuboid(position, rotation, Size, UltiDraw.Black);
            for(int i=0; i<Points.Length; i++) {
                if(Occupancies[i] > 0f) {
                    Color color = Distance[i]<threshold? colission:free;
                    UltiDraw.DrawCuboid(References[i], rotation, step, color);
                }
            }
		}
        UltiDraw.End();
	}

    public void DrawVoxelByDistance(Color color, float threshold){
        Vector3 position = Pivot.GetPosition();
        Quaternion rotation = Pivot.GetRotation();
        UltiDraw.Begin();
        Vector3 step = GetStep();
        if(Size != Vector3.zero) {
            UltiDraw.DrawWireCuboid(position, rotation, Size, UltiDraw.Black);
            for(int i=0; i<Points.Length; i++) {
                if(Occupancies[i] < threshold) {
                    UltiDraw.DrawCuboid(References[i], rotation, step, color);
                }
            }
        }
        UltiDraw.End();
    }

    public void DrawReferences() {
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Pivot.GetRotation();
        UltiDraw.Begin();
        Vector3 step = GetStep();
        if(step != Vector3.zero) {
            Color reference = UltiDraw.Black.Transparent(0.05f);
            for(int i=0; i<Points.Length; i++) {
                UltiDraw.DrawCuboid(References[i], rotation, step, reference);
            }
        }
        UltiDraw.End();
    }

    public void DrawDistribution(Color color, UltiDraw.GUIRect rect) {
        UltiDraw.Begin();
        UltiDraw.DrawGUIFunction(rect.GetPosition(), rect.GetSize(), Occupancies, 0f, 1f, UltiDraw.White, color);
        UltiDraw.End();
    }
}