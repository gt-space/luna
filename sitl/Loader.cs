using System;
using System.Reflection;

public class Loader
{
  static Loader()
  {
    string? dllPath = Environment.GetEnvironmentVariable("SITL_RELEASE");

    if (string.IsNullOrEmpty(dllPath))
    {
      Console.WriteLine("[Loader] Error: ");
      return;
    }

    try
    {
      Console.WriteLine($"[Loader] Loading peripherals from {dllPath}.");
      Assembly.LoadFrom(dllPath);
      Console.WriteLine("[Loader] DLL loaded.");
    }
    catch (Exception ex)
    {
      Console.WriteLine($"{ex.Message}");
    }
  }
}
