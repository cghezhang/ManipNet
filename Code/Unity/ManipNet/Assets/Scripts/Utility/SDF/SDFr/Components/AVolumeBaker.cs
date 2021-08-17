using System.Collections.Generic;
using UnityEngine;

namespace SDFr
{
    public abstract class AVolumeBaker<T,D> : MonoBehaviour 
        where T : AVolume<T>, new()
        where D : AVolumeData
    {
        [SerializeField] protected Bounds bounds;
        [SerializeField] protected Vector3Int dimensions = new Vector3Int(32,32,32);
        [SerializeField] protected bool fitToVertices = true;
        [SerializeField] protected List<Renderer> bakedRenderers;
        // [SerializeField] float targetVoxelSize;
        // [SerializeField] protected bool useTargetVoxelSize = false;
		[SerializeField] protected bool useStandardBorder; //invert sign of preview

		public static bool showAllPreviews = false;

        public abstract int MaxDimension { get; }

        public Vector3Int GetDimension(){
            return dimensions;
        }
        public void SetDimension(Vector3Int dim){
            dimensions = dim;
        }
        public Bounds GetBounds(){
            return bounds;
        }
        

        // for testing delete later
        public void SetBoundsCenter(Vector3 center){
            bounds.center = center;
        }
        public void SetBounds(Vector3 size){
            bounds.size = size;
        }
        // public abstract AVolumePreview<D> CreatePreview();


		// public void TogglePreview()
        // {
        //     if ( !RenderersEnabled ) EnableRenderers( true );
            
        //     if (_aPreview == null)
        //     {
        //         _aPreview = CreatePreview();
        //         EnableRenderers( false );
        //     }
        //     else
        //     {
        //         _aPreview?.Dispose();
        //         _aPreview = null;

        //        EnableRenderers( true );
        //     }
        // }

        #if UNITY_EDITOR            
        protected void EnableRenderers( bool visible )
        {
            if (bakedRenderers == null || bakedRenderers.Count == 0) return;
            RenderersEnabled = visible;
            foreach (var r in bakedRenderers)
            {
                if (r == null) continue;
                r.enabled = RenderersEnabled;
            }
        }

        /// <summary>
        /// changes bound size automatically
        /// </summary>
        public virtual void Encapsulate()
        {
            List<Renderer> tempRenderers = new List<Renderer>();
            AVolumeSettings settings = new AVolumeSettings(bounds, dimensions, useStandardBorder);
            GetMeshRenderersInChildren(ref settings, ref tempRenderers, transform, fitToVertices);
            bounds = settings.BoundsLocal;
        }
        
        public static bool GetMeshRenderersInChildren( ref AVolumeSettings settings, ref List<Renderer> renderers, Transform target, bool fitToVertices )
        {
            if (renderers == null) return false;
            
            //get renderers in children
            Renderer[] mrs = target.GetComponentsInChildren<Renderer>();
            if (mrs == null || mrs.Length == 0) return false;

            Bounds newBounds = new Bounds(target.position,Vector3.zero);
            bool first = true;
            foreach (var r in mrs)
            {
                if (!(r is MeshRenderer) && !(r is SkinnedMeshRenderer)) continue;
                //skip inactive renderers
                if (!r.gameObject.activeSelf) continue;
                if (!r.enabled) continue;

                if (!fitToVertices)
                {
                    newBounds.Encapsulate(r.bounds);
                }
                else
                {
                    //iterate all vertices and encapsulate
                    Mesh mesh = null;
                    if (r is MeshRenderer)
                    {
                        MeshFilter mf = r.GetComponent<MeshFilter>();
                        if (mf != null && mf.sharedMesh != null)
                        {
                            mesh = mf.sharedMesh;
                        }
                    }
                    else
                    {
                        mesh = (r as SkinnedMeshRenderer).sharedMesh;
                    }

                    if (mesh != null)
                    {
                        Bounds b = EncapsulateVertices(mesh, r.transform.localToWorldMatrix);

                        if (first)
                        {
                            newBounds = b;
                        }
                        else
                        {
                            newBounds.Encapsulate(b);
                        }
                        
                        first = false;
                    }
                }

                renderers.Add(r);
            }
            
            if (renderers.Count == 0)
            {
                return false;
            }

			Debug.Log( $"GetMeshRenderersInChildren newBounds: {newBounds.center}  Size: {newBounds.size}  Dimensions: {settings.Dimensions}");

            //assign new bounds
            //remove the world offset            
            newBounds = new Bounds(newBounds.center - target.position, newBounds.size);

            settings = new AVolumeSettings(newBounds, settings.Dimensions, settings.StandardBorder );
            AVolumeSettings.AddBoundsBorder(ref settings);

            return true;
        }
        
        public static bool GetMeshRenderersIntersectingVolume( AVolumeSettings settings, Transform transform, ref List<Renderer> renderers )
        {
            if (renderers == null) return false;
            
            Renderer[] mrs = FindObjectsOfType<Renderer>();
            if (mrs == null || mrs.Length == 0)
            {
                return false;
            }

            //shift bounds local to world space position
            Bounds ws = settings.BoundsLocal;
            ws.center += transform.position;
            
            foreach (var r in mrs)
            {             
                if (!(r is MeshRenderer) && !(r is SkinnedMeshRenderer)) continue;
                //skip inactive renderers
                if (!r.gameObject.activeSelf) continue;
                if (!r.enabled) continue;
                if (!ws.Intersects(r.bounds)) continue;
                renderers.Add(r);
            }

            if (renderers.Count == 0)
            {
                return false;
            }

            return true;
        }

        public static Bounds EncapsulateVertices( Mesh mesh, Matrix4x4 localToWorld )
        {
            Vector3[] vertices = new Vector3[mesh.vertexCount];
            mesh.vertices.CopyTo(vertices,0);
            
            Vector3 vert = localToWorld.MultiplyPoint3x4(vertices[0]);
            Bounds b = new Bounds(vert,Vector3.zero);
            for ( int i=1; i<vertices.Length; i++)
            {
                b.Encapsulate(localToWorld.MultiplyPoint3x4(vertices[i]));
            }

            return b;
        }
        
        private static Color colorPreviewBounds = new Color(0.5f,1f,0.5f,0.5f);
    
        protected AVolumePreview<D> _aPreview;
        public bool IsPreviewing => _aPreview != null;
        public bool RenderersEnabled { get; protected set; }
        
        private void OnValidate()
        {
            //need to ensure that bounds are not zero, and dimensions are at least something reasonable
            if (bounds.extents.x < float.Epsilon) bounds.extents = new Vector3(float.Epsilon,bounds.extents.y,bounds.extents.z);
            if (bounds.extents.y < float.Epsilon) bounds.extents = new Vector3(bounds.extents.x,float.Epsilon,bounds.extents.z);
            if (bounds.extents.z < float.Epsilon) bounds.extents = new Vector3(bounds.extents.x,bounds.extents.y,float.Epsilon);;

            // if (useTargetVoxelSize)
            // {
            //     targetVoxelSize = Mathf.Max(float.Epsilon, targetVoxelSize);
                
            //     dimensions.x = Mathf.RoundToInt(bounds.size.x / targetVoxelSize);
            //     dimensions.y = Mathf.RoundToInt(bounds.size.y / targetVoxelSize);
            //     dimensions.z = Mathf.RoundToInt(bounds.size.z / targetVoxelSize);
            // }
            
            dimensions.x = Mathf.Clamp(dimensions.x, 1, MaxDimension);
            dimensions.y = Mathf.Clamp(dimensions.y, 1, MaxDimension);
            dimensions.z = Mathf.Clamp(dimensions.z, 1, MaxDimension);
        }
        
        public abstract void Bake();

        protected virtual void OnDrawGizmos()
        {
			if ( showAllPreviews ) DrawGizmos( true, false );
			/*
            //draw bounds to indicate preview
            //ignore scale of transform
            var transform1 = transform;
            Gizmos.matrix = Matrix4x4.TRS(transform1.position, transform1.rotation, Vector3.one);
            Gizmos.color = colorPreviewBounds;
            Gizmos.DrawWireCube(bounds.center, bounds.size);
			*/
        }

        private void OnDrawGizmosSelected()
        {
			DrawGizmos( !showAllPreviews, true );
/*
            //draw voxel size
            var transform1 = transform;
            Gizmos.matrix = Matrix4x4.TRS(transform1.position, transform1.rotation, Vector3.one);
            Gizmos.color = Color.red;
            Vector3 voxelSize = new Vector3( bounds.size.x/dimensions.x, bounds.size.y/dimensions.y, bounds.size.z/dimensions.z);
            Bounds voxelBounds = new Bounds(bounds.center - bounds.extents + (voxelSize*0.5f), voxelSize);
            Gizmos.DrawWireCube(voxelBounds.center, voxelBounds.size);
			*/
        }

		void DrawGizmos( bool showPreview, bool showVoxel )
		{
			//ignore scale of transform
            var transform1 = transform;
            Gizmos.matrix = Matrix4x4.TRS(transform1.position, transform1.rotation, Vector3.one);

			if ( showPreview )
			{
				Gizmos.color = colorPreviewBounds;
				Gizmos.DrawWireCube( bounds.center, bounds.size );
			}

			if ( showVoxel )
			{
				Gizmos.color = Color.red;
				Vector3 voxelSize = new Vector3( bounds.size.x/dimensions.x, bounds.size.y/dimensions.y, bounds.size.z/dimensions.z);
				Bounds voxelBounds = new Bounds(bounds.center - bounds.extents + (voxelSize*0.5f), voxelSize);
				Gizmos.DrawWireCube(voxelBounds.center, voxelBounds.size);
			}
		}
#endif
        
    }
}