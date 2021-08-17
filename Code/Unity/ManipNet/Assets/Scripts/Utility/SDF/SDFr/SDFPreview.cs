using UnityEngine;

namespace SDFr
{
    public class SDFPreview : AVolumePreview<SDFData>
    {
        private bool _disposing;
        
        private static readonly int _SDFVolumeLocalToWorld = Shader.PropertyToID("_SDFVolumeLocalToWorld");
        private static readonly int _SDFVolumeWorldToLocal = Shader.PropertyToID("_SDFVolumeWorldToLocal");
        private static readonly int _SDFVolumeFlip = Shader.PropertyToID("_SDFVolumeFlip");
        private static readonly int _BlitScaleBiasRt = Shader.PropertyToID("_BlitScaleBiasRt");
        private static readonly int _BlitScaleBias = Shader.PropertyToID("_BlitScaleBias");
        private static readonly int _SDFPreviewEpsilon = Shader.PropertyToID("_SDFPreviewEpsilon");
        private static readonly int _SDFPreviewNormalDelta = Shader.PropertyToID("_SDFPreviewNormalDelta");

        public Texture3D debugTex3D; //for viewing existing texture3D not baked with SDFr
        
        public SDFPreview(SDFData sdfData, Shader shader, Transform transform) : base(sdfData,shader,transform)
        {
        }

        public void Draw( Camera camera, bool flip, float epsilon, float normalDelta )
        {
            if (_disposing) return;
            if (camera == null) return;
            if (_data == null) return;
            if (_props == null) return;
            if (_cmd == null) return;
            if (_material == null) return; 
            
            _data.SetMaterialProperties(_props);

            if (debugTex3D != null)
            {
                _props.SetTexture("_SDFVolumeTex",debugTex3D);
            }
            
            _props.SetMatrix(_SDFVolumeLocalToWorld, LocalToWorldNoScale);
            _props.SetMatrix(_SDFVolumeWorldToLocal, LocalToWorldNoScale.inverse);
            _props.SetFloat(_SDFVolumeFlip, flip ? -1f : 1f); 
            _props.SetVector(_BlitScaleBiasRt,new Vector4(1f,1f,0f,0f));
            _props.SetVector(_BlitScaleBias, new Vector4(1f, 1f, 0f, 0f));
            _props.SetFloat(_SDFPreviewEpsilon,epsilon);
            _props.SetFloat(_SDFPreviewNormalDelta,normalDelta);
            
            AVolumeUtils.SetupRaymarchingMatrix(
                camera.fieldOfView,
                camera.worldToCameraMatrix,
                new Vector2(camera.pixelWidth, camera.pixelHeight));
            
            _cmd.Clear();
            _cmd.DrawProcedural(Matrix4x4.identity, _material, 0, MeshTopology.Quads, 4, 1, _props);
            Graphics.ExecuteCommandBuffer(_cmd);
        }
    }
}