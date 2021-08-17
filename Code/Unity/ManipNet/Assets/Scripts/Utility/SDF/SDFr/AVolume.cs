using System;
using UnityEngine;

namespace SDFr
{
    /// <summary>
    /// Atlassed Volume
    /// </summary>
    public abstract class AVolume<T> : IDisposable where T : AVolume<T>, new()
    {
        public static T CreateVolume( Transform transform, AVolumeSettings settings )
        {
            T v = new T();
            v.Initialize(transform, settings);
            return v;
        }
        
        public AVolumeSettings Settings => _settings;
        
        public Bounds BoundsWorldAABB 
        {
            get
            {
                Bounds b = _settings.BoundsLocal;
                b.center += _transform.position;
                return b;
            }
        }
        
        protected AVolumeSettings _settings;
        protected Transform _transform;

        public Matrix4x4 LocalToWorldNoScale
        {
            get
            {
                if ( _transform == null ) return Matrix4x4.identity;
                return Matrix4x4.TRS(
                    _transform.position + _settings.BoundsLocal.center,
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
                    _transform.position + _settings.BoundsLocal.center,
                    _transform.rotation,
                    _transform.localScale);
            }
        }

        protected virtual void Initialize(Transform transform, AVolumeSettings settings)
        {
            _transform = transform;
            _settings = settings;

            //always add a bounds border when initializing AVolume
            AVolumeSettings.AddBoundsBorder( ref settings );
        }

        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                _transform = null;
            }
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }
    }
}