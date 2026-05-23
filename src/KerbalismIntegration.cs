using System;
using System.IO;
using System.Reflection;
using UnityEngine;

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    internal static class KerbalismIntegration
    {
        private static Func<Vessel, bool> _stormInProgress;
        private static bool _initialized;
        private static bool _available;

        public static bool IsAvailable => _available;

        public static void Initialize()
        {
            if (_initialized) return;
            _initialized = true;

            try
            {
                Type apiType = FindKerbalismApiType();
                if (apiType == null)
                {
                    Debug.Log("[OrbitalDecay] Kerbalism not detected — solar storm decay disabled.");
                    return;
                }

                MethodInfo method = apiType.GetMethod(
                    "StormInProgress",
                    BindingFlags.Public | BindingFlags.Static,
                    null,
                    new[] { typeof(Vessel) },
                    null);

                if (method == null)
                {
                    Debug.LogWarning("[OrbitalDecay] Kerbalism API found but StormInProgress method is missing.");
                    return;
                }

                _stormInProgress = (Func<Vessel, bool>)Delegate.CreateDelegate(typeof(Func<Vessel, bool>), method);
                _available = true;
                Debug.Log("[OrbitalDecay] Kerbalism detected — solar storm decay enabled.");
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[OrbitalDecay] Failed to initialize Kerbalism integration: " + ex.Message);
            }
        }

        public static bool IsStormInProgress(Vessel v)
        {
            if (!_initialized) Initialize();
            if (!_available || _stormInProgress == null || v == null) return false;

            try
            {
                return _stormInProgress(v);
            }
            catch
            {
                return false;
            }
        }

        private static Type FindKerbalismApiType()
        {
            foreach (Assembly assembly in AppDomain.CurrentDomain.GetAssemblies())
            {
                Type type = assembly.GetType("KERBALISM.API", false);
                if (type != null) return type;
            }

            Assembly loaded = TryLoadKerbalismAssemblyFromDisk();
            return loaded != null ? loaded.GetType("KERBALISM.API", false) : null;
        }

        private static Assembly TryLoadKerbalismAssemblyFromDisk()
        {
            // Kerbalism puts KerbalismBootstrap.dll and *.kbin directly under GameData/Kerbalism/
            string kerbalismDir = Path.Combine(KSPUtil.ApplicationRootPath, "GameData", "Kerbalism");
            if (!Directory.Exists(kerbalismDir)) return null;

            string[] fileNames = { "Kerbalism112.kbin", "Kerbalism112.dll", "Kerbalism.dll" };
            for (int i = 0; i < fileNames.Length; i++)
            {
                Assembly loaded = TryLoadAssemblyFrom(Path.Combine(kerbalismDir, fileNames[i]));
                if (loaded != null) return loaded;
            }

            try
            {
                string[] kbinFiles = Directory.GetFiles(kerbalismDir, "Kerbalism*.kbin");
                for (int i = 0; i < kbinFiles.Length; i++)
                {
                    Assembly loaded = TryLoadAssemblyFrom(kbinFiles[i]);
                    if (loaded != null) return loaded;
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[OrbitalDecay] Failed to scan Kerbalism directory: " + ex.Message);
            }

            return null;
        }

        private static Assembly TryLoadAssemblyFrom(string path)
        {
            if (!File.Exists(path)) return null;

            try
            {
                return Assembly.LoadFrom(path);
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[OrbitalDecay] Found Kerbalism at " + path + " but failed to load: " + ex.Message);
                return null;
            }
        }
    }
}
