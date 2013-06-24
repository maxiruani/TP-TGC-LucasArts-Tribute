using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using AlumnoEjemplos.LucasArtsTribute.VehicleModel;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using TgcViewer;
using TgcViewer.Utils.Shaders;
using TgcViewer.Utils.TgcGeometry;
using TgcViewer.Utils.TgcSceneLoader;

namespace AlumnoEjemplos.LucasArtsTribute
{
    public class CarReflection
    {
        public CarReflection(Vehicle car)
        {
            _car = car;
            effect = TgcShaders.loadEffect(GuiController.Instance.ExamplesDir + "Media\\Shaders\\BumpMapping.fx");
            if (!isConfigInitialize)
                ConfigReflection();
            isConfigInitialize = true;
        }
        public void Render()
        {
            SetValuesToMesh();
        }


        private void ConfigReflection()
        {
            _car.body.Effect = effect;
            _car.body.Technique = "BumpMappingTechnique";

            GuiController.Instance.Modifiers.addVertex3f("LightPosition", new Vector3(-300, -300, -300), new Vector3(300, 1500, 300), new Vector3(100, 700, 100));
            GuiController.Instance.Modifiers.addFloat("bumpiness", 0, 1, 1f);
            GuiController.Instance.Modifiers.addColor("lightColor", Color.White);
            GuiController.Instance.Modifiers.addFloat("lightIntensity", 0, 200, 150);
            GuiController.Instance.Modifiers.addFloat("lightAttenuation", 0.1f, 2, 0.3f);
            GuiController.Instance.Modifiers.addFloat("specularEx", 0, 20, 9f);
        }

        private void SetValuesToMesh()
        {
            Vector3 lightPos = (Vector3)GuiController.Instance.Modifiers["LightPosition"];
            Vector3 eyePosition = GuiController.Instance.FpsCamera.getPosition();
            /*
             * 
             * 
             */
            //Cargar variables shader de la luz
            _car.body.Effect.SetValue("lightColor", ColorValue.FromColor((Color)GuiController.Instance.Modifiers["lightColor"]));
            _car.body.Effect.SetValue("lightPosition", TgcParserUtils.vector3ToFloat4Array(lightPos));
            _car.body.Effect.SetValue("eyePosition", TgcParserUtils.vector3ToFloat4Array(eyePosition));
            _car.body.Effect.SetValue("lightIntensity", (float)GuiController.Instance.Modifiers["lightIntensity"]);
            _car.body.Effect.SetValue("lightAttenuation", (float)GuiController.Instance.Modifiers["lightAttenuation"]);
            _car.body.Effect.SetValue("bumpiness", (float)GuiController.Instance.Modifiers["bumpiness"]);

            //Material
            _car.body.Effect.SetValue("materialSpecularExp", (float)GuiController.Instance.Modifiers["specularEx"]);
            _car.body.Effect.SetValue("materialEmissiveColor", ColorValue.FromColor(Color.Gray));
            _car.body.Effect.SetValue("materialAmbientColor", ColorValue.FromColor(Color.White));
            _car.body.Effect.SetValue("materialDiffuseColor", ColorValue.FromColor(Color.White));
            _car.body.Effect.SetValue("materialSpecularColor", ColorValue.FromColor(Color.White));
            

        }

        private static bool isConfigInitialize;
        private Vehicle _car;
        private Effect effect;
    }
}
