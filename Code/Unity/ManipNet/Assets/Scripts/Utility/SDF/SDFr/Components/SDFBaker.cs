using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace SDFr
{
	public enum Visualisation {  Normal, IntensitySteps, HeatmapSteps, Distance }

    [ExecuteInEditMode] //required for previewing
    public class SDFBaker : AVolumeBaker<SDFVolume,SDFData>
    {
        [SerializeField] private int raySamples = 256;
        [SerializeField] private int jitterSeed = 555;
        [SerializeField] private float jitterScale = 0.75f;
        [SerializeField] private bool sdfFlip; //invert sign of preview
        [SerializeField] private float previewEpsilon = 0.003f;
        [SerializeField] private float previewNormalDelta = 0.02f;
        [SerializeField] private Visualisation previewMode = Visualisation.Normal;
		
        [SerializeField] private Texture3D debugTex3D; //for viewing existing texture3D not baked with SDFr
        [SerializeField] private float[] worldDistance;

		public ComputeShader  volumeComputeMethodsShader;

        public override int MaxDimension => 256;



        public float[] GetDistances(){
            // should never return the array directly, because later could be modified
            // return worldDistance
            float[] distances = new float[worldDistance.Length];
            for(int i=0; i<distances.Length; i++){
                distances[i] = worldDistance[i];
            }
            return distances;
        }

        private const string _sdfPreviewShaderName = "XRA/SDFr";
        private static Shader _shader; //TODO better way 


        #if UNITY_EDITOR    
        
        public override void Bake()
        {
            if (bakedRenderers == null)
            {
                bakedRenderers = new List<Renderer>();
            }
            bakedRenderers.Clear();
                        
            AVolumeSettings settings = new AVolumeSettings(bounds, dimensions, useStandardBorder);
            
            //first check if any objects are parented to this object
            //if anything is found, try to use renderers from those instead of volume overlap
            if (!SDFBaker.GetMeshRenderersInChildren( ref settings, ref bakedRenderers, transform, fitToVertices))
            {
                //otherwise try to get renderers intersecting the volume
                //get mesh renderers within volume
                if (!SDFBaker.GetMeshRenderersIntersectingVolume( settings, transform, ref bakedRenderers))
                {
                    //TODO display error?
                    return;
                }
            }

            SDFVolume sdfVolume = AVolume<SDFVolume>.CreateVolume(transform, settings);
            
            sdfVolume.Bake( raySamples, bakedRenderers, BakeComplete );
            
            sdfVolume.Dispose();
        }
        
        //TODO improve asset saving 
        private void BakeComplete( SDFVolume sdfVolume, float[] distances, float maxDistance, object passthrough )
        {
            //update the bounds since they may have been adjusted during bake
            bounds = sdfVolume.Settings.BoundsLocal;
            worldDistance = new float[distances.Length];
            for (int i = 0; i < distances.Length; i++)
            {
                //NOTE for compatibility with Visual Effect Graph, 
                //the distance must be negative inside surfaces.
                //normalize the distance for better support of scaling bounds
                //Max Distance is always the Magnitude of the baked bound size
                
                worldDistance[i] = distances[i];
                // float normalizedDistance = distances[i] / maxDistance;
                // normdDistances[i] = normalizedDistance; 
            }
          
        }

        /// <summary>
        /// renders a procedural quad
        /// TODO fit the bounds of the SDF Volume
        /// </summary>
        private void OnRenderObject()
        {
            if (!IsPreviewing) return;
            
            //try to get active camera...
            Camera cam = Camera.main;

            // lastActiveSceneView
            if (SceneView.currentDrawingSceneView != null) cam = SceneView.currentDrawingSceneView.camera;
            
            SDFPreview preview = _aPreview as SDFPreview;
            
            preview?.Draw(cam,sdfFlip,previewEpsilon,previewNormalDelta);
        }
        #endif
    }
}