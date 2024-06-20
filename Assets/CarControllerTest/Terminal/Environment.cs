using System;
using UnityEngine;

namespace CarControllerTest.Terminal
{
    public class Environment<T> : MonoBehaviour where T : Environment<T>
    {
        private static T _instance;

        public static T Instance
        {
            get => _instance;
            private set => _instance = value;
        }

        protected void Awake()
        {
            _instance = this as T;
        }
        
        
    }
}