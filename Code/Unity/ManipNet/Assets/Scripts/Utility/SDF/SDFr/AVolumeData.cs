using UnityEngine;

namespace SDFr
{
    public struct AVolumeSettings
    {
        public readonly Vector3Int Dimensions;
        public readonly Bounds BoundsLocal;
        
        public int CellCount { get; }
        public int MaxDimension { get; }
        public Vector3 VoxelSize { get; }
        public Vector3 HalfVoxel => VoxelSize * 0.5f;
		public bool StandardBorder { get; }

        public AVolumeSettings(Bounds boundsLocal, Vector3Int dimensions, bool standardBorder)
        {
			StandardBorder = standardBorder;
            BoundsLocal = boundsLocal;
            Dimensions = dimensions;
            CellCount = Dimensions.x * Dimensions.y * Dimensions.z;
            MaxDimension = Mathf.Max(Dimensions.x, Dimensions.y, Dimensions.z);
            VoxelSize = new Vector3(
                BoundsLocal.size.x/Dimensions.x,
                BoundsLocal.size.y/Dimensions.y,
                BoundsLocal.size.z/Dimensions.z);

			Debug.Log( $"AVolumeSettings BCenter {BoundsLocal.center} BSize: {BoundsLocal.size} Dim: {Dimensions}  Cells: {CellCount} MaxDim: {MaxDimension} VoxelSize: {VoxelSize.ToString("F3")}");

        }
        
        public Vector3Int FromIndex(int index)
        {
            int i = index;
            int zz = i / (Dimensions.x*Dimensions.y);
            i -= zz * Dimensions.x*Dimensions.y;
            int yy = i / Dimensions.x;
            int xx = i % Dimensions.x;
            return new Vector3Int(xx,yy,zz);
        }
        
        public Vector3 ToPositionWS(int index, Matrix4x4 localToWorld)
        {
            Vector3Int xyz = FromIndex(index);

            //0 to 1 normalized bound space
            Vector3 positionBS = new Vector3(
                xyz.x/(float)Dimensions.x-0.5f,
                xyz.y/(float)Dimensions.y-0.5f,
                xyz.z/(float)Dimensions.z-0.5f
            );
            //scale by bound size
            positionBS = Vector3.Scale(positionBS, BoundsLocal.size);
            //transform by the local to world matrix
            //NOTE scale cannot be used here otherwise it would double scale the bounds
            //(which have already encapsulated renderers in world space)
            return localToWorld.MultiplyPoint3x4(positionBS);
        }
        
        public static void AddBoundsBorder( ref AVolumeSettings settings )
        {			
            //divide current by dimensions-2 so that it gets a voxel size without the borders
            Vector3 extraBorderVoxelSize = new Vector3(
                settings.BoundsLocal.size.x / Mathf.Max(1,settings.Dimensions.x - 2),
                settings.BoundsLocal.size.y / Mathf.Max(1,settings.Dimensions.y - 2),
                settings.BoundsLocal.size.z / Mathf.Max(1,settings.Dimensions.z - 2));

			// Just testing if we can minimise the extraBorder bounds to not lose a voxel - but not sure it does any good.
			if ( !settings.StandardBorder )
			{
				// float offset = 0.5f;
				// extraBorderVoxelSize = new Vector3( offset, offset, offset ) * 0.5f;

                // here i modified, when not use standard border, just fit to the mesh
                extraBorderVoxelSize = Vector3.zero;
			}

            //then add extra voxel size so that bordering voxels are outside original bounds
            Bounds newBounds = new Bounds(settings.BoundsLocal.center, settings.BoundsLocal.size + extraBorderVoxelSize * 2f);            
			Debug.Log( $"AddBoundsBorder Bounds: {newBounds.center}  Size: {newBounds.size}  Dimensions: {settings.Dimensions}  StandardBorder: {settings.StandardBorder}");
			
            settings = new AVolumeSettings(newBounds, settings.Dimensions, settings.StandardBorder );
        }
    }
    
    public abstract class AVolumeData : ScriptableObject
    {
        public Vector3Int dimensions;
        /// <summary>
        /// Local bounds
        /// </summary>
        public Bounds bounds;
        public Vector3 voxelSize;
        public Vector3 nonUniformScale;
    }
}