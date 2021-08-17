using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace SDFr.Editor
{
    public abstract class AVolumeBakerInspector<T,V,D> : UnityEditor.Editor 
        where V : AVolume<V>, new()
        where T : AVolumeBaker<V,D>, new()
        where D : AVolumeData
    {
        //serialized properties
        protected SerializedProperty DimensionsProperty;
        protected SerializedProperty BoundsProperty;
        // protected SerializedProperty UseTargetVoxelSizeProperty;		
		protected SerializedProperty useStandardBorderProperty;
        protected SerializedProperty TargetVoxelSizeProperty;
		
        protected const string StrBake = "Bake";
        protected const string StrPreview = "Preview";
        protected const string StrEndPreview = "End Preview";
        protected const string StrHideRenderers = "Hide Baked Renderers";
        protected const string StrShowRenderers = "Show Baked Renderers";
        protected const string StrPropDimensions = "dimensions";
        protected const string StrPropBounds = "bounds";
        // protected const string StrPropUseTargetVoxelSize = "useTargetVoxelSize";
        // protected const string StrPropTargetVoxelSize = "targetVoxelSize";
        protected const string StrEncapsulate = "Encapsulate";        
		protected const string strPropStandardBorder = "useStandardBorder";

        [SerializeField] protected Color ColorHandles = new Color(0.5f,1f,1f,1f);
        [SerializeField] protected Color ColorWires = new Color(0.5f,1f,0.5f,1f);
        
        protected BoxBoundsHandle BoxBoundsHandle;

        protected virtual void CollectSerializedProperties()
        {
            //collect serialized properties
            BoundsProperty = serializedObject.FindProperty(StrPropBounds);
            DimensionsProperty = serializedObject.FindProperty(StrPropDimensions);
            // UseTargetVoxelSizeProperty = serializedObject.FindProperty(StrPropUseTargetVoxelSize);
			useStandardBorderProperty = serializedObject.FindProperty(strPropStandardBorder);
            // TargetVoxelSizeProperty = serializedObject.FindProperty(StrPropTargetVoxelSize);
        }
        
        protected virtual void OnEnable()
        {
            if ( BoxBoundsHandle == null ) BoxBoundsHandle = new BoxBoundsHandle();
        
            CollectSerializedProperties();
        }
        
        protected virtual void OnDisable()
        {
            BoxBoundsHandle = null;
        }
                
        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            DrawVolumeGUI();
            
            if (serializedObject.ApplyModifiedProperties())
            {
                //no settings are used until re-baking
            }
        }
    
        protected virtual void DrawVolumeGUI()
        {
            DrawBaseGUI();
        }

        protected void DrawBaseGUI()
        {
            // EditorGUILayout.PropertyField(UseTargetVoxelSizeProperty);
            // bool useTargetVoxelSize = UseTargetVoxelSizeProperty.boolValue;
            
            // // target voxel size (uniform)
            // if (useTargetVoxelSize) EditorGUILayout.PropertyField(TargetVoxelSizeProperty);
            			
			EditorGUILayout.PropertyField(useStandardBorderProperty);

            //dimensions, if using target voxel size show dimensions as non editable
            // EditorGUI.BeginDisabledGroup(useTargetVoxelSize);
            EditorGUILayout.PropertyField(DimensionsProperty);
            EditorGUI.EndDisabledGroup();

            //bounds
            EditorGUILayout.PropertyField(BoundsProperty);
            
            if (GUILayout.Button(StrEncapsulate))
            {
                T baker = target as T;
                if (baker != null)
                {
                    baker.Encapsulate();
                }
            }
            
        }

        protected virtual void BakeControls()
        {
            T baker = target as T;
            if (baker == null) return;
            if (GUILayout.Button(StrBake))
            {
                baker.Bake();
            }
    
            // if (GUILayout.Button((baker.IsPreviewing) ? StrEndPreview : StrPreview))
            // {
            //     baker.TogglePreview();
            //     SceneView.RepaintAll();
            // }
        }
        
        public void OnSceneGUI()
        {
			if (Selection.activeGameObject == null) return;

            DrawEditableBounds();
        }
        
        private void DrawEditableBounds()
        {
            //draw editable bounds
            EditorGUI.BeginChangeCheck();
        
            Transform st = Selection.activeGameObject.transform;
            
            //ignore scale of transform
            Matrix4x4 localToWorld = Matrix4x4.TRS(st.position, st.rotation, Vector3.one);

            Handles.matrix = localToWorld;
            BoxBoundsHandle.handleColor = ColorHandles;
            BoxBoundsHandle.wireframeColor = ColorWires;
            BoxBoundsHandle.center = BoundsProperty.boundsValue.center;
            BoxBoundsHandle.size = BoundsProperty.boundsValue.size;
            BoxBoundsHandle.DrawHandle();

            if (!EditorGUI.EndChangeCheck()) return;
        
            //update bounds
            BoundsProperty.boundsValue = new Bounds
            {
                center = BoxBoundsHandle.center,
                size = BoxBoundsHandle.size
            };
                
            //draw voxel size
            Gizmos.color = Color.red;
            Vector3 voxelSize = new Vector3( 
                BoundsProperty.boundsValue.size.x/DimensionsProperty.vector3IntValue.x, 
                BoundsProperty.boundsValue.size.y/DimensionsProperty.vector3IntValue.y, 
                BoundsProperty.boundsValue.size.z/DimensionsProperty.vector3IntValue.z);
            Bounds voxelBounds = new Bounds(BoundsProperty.boundsValue.center - BoundsProperty.boundsValue.extents + (voxelSize*0.5f), voxelSize);
            Gizmos.DrawWireCube(voxelBounds.center, voxelBounds.size);

            if (serializedObject.ApplyModifiedProperties())
            {
                //no settings applied until re-baking
            }
        }
    }
}