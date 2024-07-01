using UnityEngine;
using UnityEditor;
using System;

namespace AshVP
{
    public class VehicleCreator : EditorWindow
    {

        GameObject preset;
        Transform VehicleParent;
        Transform wheelFL;
        Transform wheelFR;
        Transform wheelRL;
        Transform wheelRR;

        //bool isBike = false;
        Transform frontWheel,backWheel;

        MeshRenderer bodyMesh;
        MeshRenderer wheelMesh;

        private GameObject NewVehicle;


        [MenuItem("Tools/Ash Vehicle Physics/Vehicle Creator")]

        static void OpenWindow()
        {
            VehicleCreator vehicleCreatorWindow = (VehicleCreator)GetWindow(typeof(VehicleCreator));
            vehicleCreatorWindow.minSize = new Vector2(400, 300);
            vehicleCreatorWindow.Show();
        }


        private void OnGUI()
        {
            var style = new GUIStyle(EditorStyles.boldLabel);
            style.normal.textColor = Color.green;
            GUILayout.Label("Ash Vehicle Creator", style);

            //isBike = EditorGUILayout.Toggle("Is Bike", isBike);

            //if (isBike)
            //{
            //    preset = EditorGUILayout.ObjectField("Bike preset", preset, typeof(GameObject), true) as GameObject;
            //    GUILayout.Label("Your Vehicle", style);
            //    VehicleParent = EditorGUILayout.ObjectField("Bike Parent", VehicleParent, typeof(Transform), true) as Transform;
            //
            //    GUILayout.Label("Wheels", style);
            //
            //    frontWheel = EditorGUILayout.ObjectField("Front Wheel", frontWheel, typeof(Transform), true) as Transform;
            //    backWheel = EditorGUILayout.ObjectField("Back Wheel", backWheel, typeof(Transform), true) as Transform;
            //    if (GUILayout.Button("Create Bike"))
            //    {
            //        createBike();
            //    }
            //}
            //else
            //{
                preset = EditorGUILayout.ObjectField("Vehicle preset", preset, typeof(GameObject), true) as GameObject;
                GUILayout.Label("Your Vehicle", style);
                VehicleParent = EditorGUILayout.ObjectField("Vehicle Parent", VehicleParent, typeof(Transform), true) as Transform;

                GUILayout.Label("Wheels", style);
                wheelFL = EditorGUILayout.ObjectField("wheel FL", wheelFL, typeof(Transform), true) as Transform;
                wheelFR = EditorGUILayout.ObjectField("wheel FR", wheelFR, typeof(Transform), true) as Transform;
                wheelRL = EditorGUILayout.ObjectField("wheel RL", wheelRL, typeof(Transform), true) as Transform;
                wheelRR = EditorGUILayout.ObjectField("wheel RR", wheelRR, typeof(Transform), true) as Transform;

                if (GUILayout.Button("Create Vehicle"))
                {
                    createVehicle();
                }
            //}

            bodyMesh = EditorGUILayout.ObjectField("body Mesh", bodyMesh, typeof(MeshRenderer), true) as MeshRenderer;
            wheelMesh = EditorGUILayout.ObjectField("wheel Mesh", wheelMesh, typeof(MeshRenderer), true) as MeshRenderer;

            if (GUILayout.Button("Adjust Colliders"))
            {
                adjustColliders();
            }

        }

        private void adjustColliders()
        {
            if (NewVehicle.GetComponent<BoxCollider>())
            {
                NewVehicle.GetComponent<BoxCollider>().center = Vector3.zero;
                NewVehicle.GetComponent<BoxCollider>().size = bodyMesh.bounds.size;
            }

            if (NewVehicle.GetComponent<CapsuleCollider>())
            {
                NewVehicle.GetComponent<CapsuleCollider>().center = Vector3.zero;
                NewVehicle.GetComponent<CapsuleCollider>().height = bodyMesh.bounds.size.z;
                NewVehicle.GetComponent<CapsuleCollider>().radius = bodyMesh.bounds.size.x / 2;

            }

            Vector3 SpheareColliderOffset = new Vector3(wheelMesh.bounds.extents.x, 0, 0);
            //if (isBike) { SpheareColliderOffset = Vector3.zero; }

            foreach (Transform wheel in NewVehicle.transform.Find("wheels"))
            {
                wheel.GetComponent<SphereCollider>().radius = wheelMesh.bounds.extents.y;
                wheel.GetComponent<SphereCollider>().center = SpheareColliderOffset;
            }

        }

        private void createVehicle()
        {
            Make_Vehicle_Ready_For_Setup();

            var vehiclePos = Vector3.zero;
            if (bodyMesh != null)
            {
                vehiclePos = bodyMesh.bounds.center;
            }
            else
            {
                vehiclePos = VehicleParent.position;
            }
            NewVehicle = Instantiate(preset, vehiclePos, VehicleParent.rotation);
            NewVehicle.name = "Ash_" + VehicleParent.name;

            GameObject.DestroyImmediate(NewVehicle.transform.Find("body").Find("mesh body").GetChild(0).gameObject);
            if (NewVehicle.transform.Find("wheels").Find("FL rb"))
            {
                GameObject.DestroyImmediate(NewVehicle.transform.Find("wheels").Find("FL rb").Find("FL").Find("Wmesh.FL").GetChild(0).gameObject);
            }
            if (NewVehicle.transform.Find("wheels").Find("FR rb"))
            {
                GameObject.DestroyImmediate(NewVehicle.transform.Find("wheels").Find("FR rb").Find("FR").Find("Wmesh.FR").GetChild(0).gameObject);
            }
            if (NewVehicle.transform.Find("wheels").Find("RL rb"))
            {
                GameObject.DestroyImmediate(NewVehicle.transform.Find("wheels").Find("RL rb").Find("RL").Find("Wmesh.RL").GetChild(0).gameObject);
            }
            if (NewVehicle.transform.Find("wheels").Find("RR rb"))
            {
                GameObject.DestroyImmediate(NewVehicle.transform.Find("wheels").Find("RR rb").Find("RR").Find("Wmesh.RR").GetChild(0).gameObject);
            }
            NewVehicle.transform.Find("body").localPosition = Vector3.zero;
            VehicleParent.parent = NewVehicle.transform.Find("body").Find("mesh body");
            //VehicleBody.localPosition = Vector3.zero;
            NewVehicle.transform.Find("wheels").position = vehiclePos;

            if (NewVehicle.transform.Find("wheels").Find("FL rb"))
            {
                NewVehicle.transform.Find("wheels").Find("FL rb").position = wheelFL.position;
                wheelFL.parent = NewVehicle.transform.Find("wheels").Find("FL rb").Find("FL").Find("Wmesh.FL");
            }
            if (NewVehicle.transform.Find("wheels").Find("FR rb"))
            {
                NewVehicle.transform.Find("wheels").Find("FR rb").position = wheelFR.position;
                wheelFR.parent = NewVehicle.transform.Find("wheels").Find("FR rb").Find("FR").Find("Wmesh.FR");
            }
            if (NewVehicle.transform.Find("wheels").Find("RL rb"))
            {
                NewVehicle.transform.Find("wheels").Find("RL rb").position = wheelRL.position;
                wheelRL.parent = NewVehicle.transform.Find("wheels").Find("RL rb").Find("RL").Find("Wmesh.RL");
            }
            if (NewVehicle.transform.Find("wheels").Find("RR rb"))
            {
                NewVehicle.transform.Find("wheels").Find("RR rb").position = wheelRR.position;
                wheelRR.parent = NewVehicle.transform.Find("wheels").Find("RR rb").Find("RR").Find("Wmesh.RR");
            }


        }

        private void createBike()
        {
            Make_Vehicle_Ready_For_Setup();

            var vehiclePos = Vector3.zero;
            if (bodyMesh != null)
            {
                vehiclePos = bodyMesh.bounds.center;
            }
            else
            {
                vehiclePos = VehicleParent.position;
            }
            NewVehicle = Instantiate(preset, vehiclePos, VehicleParent.rotation);
            NewVehicle.name = "Ash_" + VehicleParent.name;

            GameObject.DestroyImmediate(NewVehicle.transform.Find("body").Find("mesh body").GetChild(0).gameObject);
            if (NewVehicle.transform.Find("wheels").Find("F rb"))
            {
                GameObject.DestroyImmediate(NewVehicle.transform.Find("wheels").Find("F rb").Find("F").Find("Wmesh.F").GetChild(0).gameObject);
            }
            if (NewVehicle.transform.Find("wheels").Find("R rb"))
            {
                GameObject.DestroyImmediate(NewVehicle.transform.Find("wheels").Find("R rb").Find("R").Find("Wmesh.R").GetChild(0).gameObject);
            }
            NewVehicle.transform.Find("body").localPosition = Vector3.zero;
            VehicleParent.parent = NewVehicle.transform.Find("body").Find("mesh body");
            //VehicleBody.localPosition = Vector3.zero;
            NewVehicle.transform.Find("wheels").position = vehiclePos;

            if (NewVehicle.transform.Find("wheels").Find("F rb"))
            {
                NewVehicle.transform.Find("wheels").Find("F rb").position = frontWheel.position;
                frontWheel.parent = NewVehicle.transform.Find("wheels").Find("F rb").Find("F").Find("Wmesh.F");
            }
            if (NewVehicle.transform.Find("wheels").Find("R rb"))
            {
                NewVehicle.transform.Find("wheels").Find("R rb").position = backWheel.position;
                backWheel.parent = NewVehicle.transform.Find("wheels").Find("R rb").Find("R").Find("Wmesh.R");
            }
        }

        private void Make_Vehicle_Ready_For_Setup()
        {

            var AllVehicleColliders = VehicleParent.GetComponentsInChildren<Collider>();
            foreach (var collider in AllVehicleColliders)
            {
                DestroyImmediate(collider);
            }

            var AllRigidBodies = VehicleParent.GetComponentsInChildren<Rigidbody>();
            foreach (var rb in AllRigidBodies)
            {
                DestroyImmediate(rb);
            }

        }


    }
}
