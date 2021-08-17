using System;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

namespace SDFr
{
    public abstract class AVolumePreview<Tdata> : IDisposable 
        where Tdata : AVolumeData
    {
        
        public Matrix4x4 LocalToWorldNoScale
        {
            get
            {
                if ( _transform == null ) return Matrix4x4.identity;
                return Matrix4x4.TRS(
                    _transform.position + _data.bounds.center,
                    _transform.rotation,
                    Vector3.one);
            }
        }
        
        public Matrix4x4 LocalToWorld
        {
            get
            {
                if ( _transform == null ) return Matrix4x4.identity;
                
                return Matrix4x4.TRS(
                    _transform.position + _data.bounds.center,
                    _transform.rotation,
                    _transform.localScale);
            }
        }
        
        protected Transform _transform;
        protected CommandBuffer _cmd;
        protected MaterialPropertyBlock _props;
        protected Material _material;
        protected Tdata _data;
        private bool _disposing;
        
        
        
        /// <summary>
        /// Creates a volume preview following the transform
        /// </summary>
        /// <param name="data"></param>
        /// <param name="shader"></param>
        /// <param name="transform"></param>
        public AVolumePreview( Tdata data, Shader shader, Transform transform )
        {
            if ( data == null || shader == null || transform == null )
            {
                Dispose();
                return;
            }

            _transform = transform;
            _data = data;
            _cmd = new CommandBuffer {name = "["+_data.GetType()+"]" + _data.name};
            _props = new MaterialPropertyBlock();
            _material = new Material(shader);
        }

        protected AVolumePreview()
        {
        }

        //public abstract void Draw();
        
        #region IDisposable

        private void ReleaseUnmanagedResources()
        {
            // TODO release unmanaged resources here
        }

        protected virtual void Dispose(bool disposing)
        {
            ReleaseUnmanagedResources();
            if (disposing)
            {
                _cmd?.Dispose();
                _props = null;
                _data = null;
                if (_material != null)
                {
                    Object.DestroyImmediate(_material);
                } 
            }
        }

        ~AVolumePreview()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }
        
        #endregion
    }
}