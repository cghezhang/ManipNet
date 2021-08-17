using UnityEditor;
using UnityEngine;

namespace SDFr.Editor
{
    //TODO add tooltips, move bake / preview to base inspector 
    [CustomEditor(typeof(SDFBaker))]
    public class SDFBakerInspector : AVolumeBakerInspector<SDFBaker,SDFVolume,SDFData>
    {
        private SerializedProperty fitToVerticesProperty;
        private SerializedProperty raySamplesProperty;
        private SerializedProperty sdfDataProperty;
        private SerializedProperty sdfFlipProperty;
        private SerializedProperty jitterSeedProperty;
        private SerializedProperty jitterScaleProperty;
        private SerializedProperty epsilonProperty;
        private SerializedProperty normalDeltaProperty;
		private SerializedProperty previewModeProperty;

        private const string strPropFitToVertices = "fitToVertices";
        private const string strPropRaySamples = "raySamples";
        private const string strPropSdfData = "sdfData";
        private const string strPropSdfFlip = "sdfFlip";
        private const string strPropJitterSeed = "jitterSeed";
        private const string strPropJitterScale = "jitterScale";
        private const string strPropEpsilon = "previewEpsilon";
        private const string strPropNormalDelta = "previewNormalDelta";
        private const string strPropPreviewMode = "previewMode";

        protected override void CollectSerializedProperties()
        {
            base.CollectSerializedProperties();
            fitToVerticesProperty = serializedObject.FindProperty(strPropFitToVertices);
            raySamplesProperty = serializedObject.FindProperty(strPropRaySamples);
            sdfDataProperty = serializedObject.FindProperty(strPropSdfData);
            sdfFlipProperty = serializedObject.FindProperty(strPropSdfFlip);
            jitterSeedProperty = serializedObject.FindProperty(strPropJitterSeed);
            jitterScaleProperty = serializedObject.FindProperty(strPropJitterScale);
            epsilonProperty = serializedObject.FindProperty(strPropEpsilon);
            normalDeltaProperty = serializedObject.FindProperty(strPropNormalDelta);
			previewModeProperty = serializedObject.FindProperty(strPropPreviewMode);
        }

        protected override void OnDisable()
        {
            BoxBoundsHandle = null;
        }

		protected override void DrawVolumeGUI()
		{
			SDFBaker baker = target as SDFBaker;
			if ( baker == null ) return;

			EditorGUI.BeginChangeCheck();

			//disable these when previewing to enforce idea that they are for baking
			EditorGUI.BeginDisabledGroup( baker.IsPreviewing );
			DrawBaseGUI();
			//ray samples
			EditorGUILayout.PropertyField( raySamplesProperty );
			//fit to vertices of mesh, if false the bounds will be used, bounds may be larger than mesh and waste space
			EditorGUILayout.PropertyField( fitToVerticesProperty );
			//jitter seed 
			//EditorGUILayout.PropertyField(jitterSeedProperty);
			//jitter scale (0.75 to 1.0 seems good)
			//EditorGUILayout.PropertyField(jitterScaleProperty);
			EditorGUI.EndDisabledGroup();

			// //disable these when not previewing 
			// EditorGUI.BeginDisabledGroup( !baker.IsPreviewing );
			// EditorGUILayout.Slider( epsilonProperty, 0.0001f, 0.005f );
			// EditorGUILayout.Slider( normalDeltaProperty, 0.0001f, 0.02f );

			// using ( var check = new EditorGUI.ChangeCheckScope() )
			// {
			// 	EditorGUILayout.PropertyField( previewModeProperty );
			// 	if ( check.changed ) SetKeywords( (Visualisation)previewModeProperty.enumValueIndex);
			// }

            // //inverse sign of SDF preview
            // EditorGUILayout.PropertyField(sdfFlipProperty);

            // EditorGUI.EndDisabledGroup();
            
            // //TODO if SDF Data assigned via drag & drop, adjust the bounds and settings to match SDF data
            // EditorGUILayout.PropertyField(sdfDataProperty);
			
            // BakeControls();
			// if (GUILayout.Button("BakeHalfSize")) baker.BakeHalfSizeTest();
			// if (GUILayout.Button("Log Volume Data")) baker.LogDistances();
        }

		void SetKeywords( Visualisation mode )
		{
			Shader.DisableKeyword( "SDFr_VISUALIZE_STEPS" );
			Shader.DisableKeyword( "SDFr_VISUALIZE_HEATMAP" );
			Shader.DisableKeyword( "SDFr_VISUALIZE_DIST" );

			switch ( mode )
			{
				case Visualisation.IntensitySteps:	Shader.EnableKeyword( "SDFr_VISUALIZE_STEPS" ); break;
				case Visualisation.HeatmapSteps:	Shader.EnableKeyword( "SDFr_VISUALIZE_HEATMAP" ); break;
				case Visualisation.Distance:		Shader.EnableKeyword( "SDFr_VISUALIZE_DIST" ); break;
			}
			SceneView.RepaintAll();
		}
    }
}