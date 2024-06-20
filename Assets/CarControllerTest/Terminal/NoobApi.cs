using System;
using UnityEngine;

namespace CarControllerTest.Terminal
{
    public class NoobApi : Environment<NoobApi>
    {
        [Serializable]
        public struct GameplaySettings
        {
            public float gravity;
        }

        public GameplaySettings Settings;
    }
}