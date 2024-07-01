using UnityEngine;
using UnityEditor;

namespace AshVP
{
    public class FreeAssetPromotionWindow : EditorWindow
    {
        private string assetName = "Rally Racing Asset";
        private string reviewInstructions = "Click the button below to leave a review for 'Ash Vehicle Physics' on the Unity Asset Store.";
        private string proofInstructions = "Then email a screenshot of your review and invoice number to ";
        private string email = "ashdevbiz@gmail.com";
        private string discordUser = "ash_dev";
        private string youtubeLink = "https://youtu.be/wD2_VyWwcgk";
        private string reviewUrl = "https://assetstore.unity.com/packages/tools/physics/ash-vehicle-physics-187803#reviews";

        private GUIStyle headerStyle;
        private GUIStyle stepHeaderStyle;
        private GUIStyle textStyle;
        private GUIStyle linkButtonStyle;
        private GUIStyle buttonStyle;
        private GUIStyle stepBoxStyle;

        [MenuItem("Tools/Ash Vehicle Physics/Get Free Asset Offer!")]
        public static void ShowWindow()
        {
            var window = GetWindow<FreeAssetPromotionWindow>("Free Asset Offer");
            window.minSize = new Vector2(500, 400); // Set a fixed minimum size for the window
        }

        void OnGUI()
        {
            InitializeStyles();

            GUILayout.Space(10);
            GUILayout.Label($"Get a Free {assetName}!", headerStyle);
            GUILayout.Space(10);

            // Step 1 Box
            EditorGUILayout.BeginVertical(stepBoxStyle);
            GUILayout.Label("Step 1: Review", stepHeaderStyle);
            GUILayout.Label(reviewInstructions, textStyle);
            if (GUILayout.Button("Leave a Review", linkButtonStyle))
            {
                Application.OpenURL(reviewUrl);
            }
            EditorGUILayout.EndVertical();
            GUILayout.Space(10);

            // Step 2 Box
            EditorGUILayout.BeginVertical(stepBoxStyle);
            GUILayout.Label("Step 2: Send Proof", stepHeaderStyle);
            GUILayout.Label(proofInstructions, textStyle);
            if (GUILayout.Button(email, linkButtonStyle))
            {
                EditorGUIUtility.systemCopyBuffer = email;
                ShowNotification(new GUIContent("Email address copied to clipboard!"));
            }
            GUILayout.Label("Or DM me on Discord:", textStyle);
            if (GUILayout.Button(discordUser, linkButtonStyle))
            {
                EditorGUIUtility.systemCopyBuffer = discordUser;
                ShowNotification(new GUIContent("Discord username copied to clipboard!"));
            }
            EditorGUILayout.EndVertical();
            GUILayout.Space(10);

            // YouTube Link Box
            if (GUILayout.Button("Check out what you will get", buttonStyle))
            {
                Application.OpenURL(youtubeLink);
            }
        }

        private void InitializeStyles()
        {
            // Optional: Create a texture for header background
            Texture2D headerBgTexture = new Texture2D(1, 1);
            headerBgTexture.SetPixel(0, 0, new Color(0.1f, 0.1f, 0.1f, 0.5f)); // Semi-transparent dark background
            headerBgTexture.Apply();

            headerStyle = new GUIStyle(EditorStyles.largeLabel)
            {
                fontSize = 32, // Increased font size for better visibility
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter,
                normal =
        {
            textColor = new Color(0.2f, 0.75f, 0.2f), // Vibrant green color
            background = headerBgTexture // Background texture for style
        },
                padding = new RectOffset(0, 0, 10, 10), // Add some padding to give space around the text
                margin = new RectOffset(0, 0, 10, 10), // Add margin to separate from other elements
                border = new RectOffset(0, 0, 0, 0) // Adjust border to control background stretch
            };

            stepHeaderStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                fontSize = 18,
                alignment = TextAnchor.MiddleLeft,
                fontStyle = FontStyle.Bold,
                normal = { textColor = new Color(0.2f, 0.75f, 0.2f) } // Green color
            };

            textStyle = new GUIStyle(EditorStyles.label)
            {
                fontSize = 14,
                alignment = TextAnchor.UpperLeft,
                fontStyle = FontStyle.Normal,
                wordWrap = true,
                normal = { textColor = EditorGUIUtility.isProSkin ? Color.white : Color.black }
            };

            linkButtonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 14,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter,
                normal = { textColor = new Color(0.11f, 0.8f, 0.8f) }, // Link color
                hover = { textColor = new Color(1f, 0.5f, 0.8f) }
            };

            buttonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 25,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter,
                normal = { textColor = Color.yellow },
                hover = { textColor = new Color(1f, 0.6f, 0f) },
                fixedHeight = 40,
                margin = new RectOffset(10, 10, 4, 4)
            };

            stepBoxStyle = new GUIStyle(GUI.skin.box)
            {
                padding = new RectOffset(15, 15, 10, 10),
                normal = { background = EditorGUIUtility.isProSkin ? Texture2D.blackTexture : Texture2D.whiteTexture },
                margin = new RectOffset(10, 10, 10, 10)
            };
        }
    }
}
