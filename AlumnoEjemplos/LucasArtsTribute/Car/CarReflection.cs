using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using AlumnoEjemplos.LucasArtsTribute.Models;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils.Shaders;
using TgcViewer.Utils.TgcGeometry;

namespace AlumnoEjemplos.LucasArtsTribute
{
    internal class CarReflection
    {
        String folder = "F:\\UTN\\TGC\\LucasArtsTribute\\TP-TGC-LucasArts-Tribute\\AlumnoEjemplos\\LucasArtsTribute\\Shader";

        public CarReflection()
        {
            /*String compilationErrors;
            
            effect = Effect.FromFile(GuiController.Instance.D3dDevice, folder + "\\PhongShading.fx", null, null,
                                     ShaderFlags.None, null, out compilationErrors);
            if (effect == null)
                return;
            effect.Technique = "DefaultTechnique";*/

            effect = TgcShaders.loadEffect(GuiController.Instance.ExamplesDir + "Shaders\\WorkshopShaders\\Shaders\\PhongShading.fx");

            // Pasos standard: 
            // le asigno el efecto a la malla 


        }

        public void EffectEnable(CorvetteCar car)
        {
            car.Mesh.Effect = effect;
            car.Mesh.Technique = "DefaultTechnique";

            GuiController.Instance.Modifiers.addVertex3f("LightPosition", new Vector3(-100, -100, -100), new Vector3(100, 100, 100), new Vector3(0, 40, 0));
            GuiController.Instance.Modifiers.addFloat("Ambient", 0, 1, 0.5f);
            GuiController.Instance.Modifiers.addFloat("Diffuse", 0, 1, 0.6f);
            GuiController.Instance.Modifiers.addFloat("Specular", 0, 1, 0.5f);
            GuiController.Instance.Modifiers.addFloat("SpecularPower", 1, 100, 16);
        }

        public void EffectDisable(CorvetteCar car)
        {
            car.Mesh.Effect = null;
        }



        private Effect effect;
    }
}
