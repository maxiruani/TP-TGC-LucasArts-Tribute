using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using TgcViewer;
using TgcViewer.Utils._2D;
using TgcViewer.Utils.TgcSceneLoader;
using System.Drawing;
using TgcViewer.Utils.TgcGeometry;

using Microsoft.DirectX.Direct3D;

namespace AlumnoEjemplos.LucasArtsTribute
{
    class Velocimetro
    {
        TgcSprite indicador, agujaVelocidad, agujaTacometro;
        string alumnoMediaFolder = GuiController.Instance.AlumnoEjemplosMediaDir;
        Size screenSize = GuiController.Instance.Panel3d.Size;

        TgcText2d textCambio;

        TgcText2d textVelocity;

        //Bandera de Aceleracion (no importa si es para adelante o atras)
        public bool acelera;
        float rotacionTaquimetro;
        float rotacionVelocimetro;

        Microsoft.DirectX.Direct3D.Sprite DxSprite;

        public Velocimetro()
        {
            //Setea la bandera de aceleracion por default en false
            acelera = false;
            rotacionTaquimetro = 0f;
            rotacionVelocimetro = 0f;
            
            //Crear Sprites
            indicador = new TgcSprite();
            indicador.Texture = TgcTexture.createTexture(alumnoMediaFolder + "LucasArtsTribute\\Velocimetro\\velocimetro.png");

            agujaVelocidad = new TgcSprite();
            agujaVelocidad.Texture = TgcTexture.createTexture(alumnoMediaFolder + "LucasArtsTribute\\Velocimetro\\agujaVelocidad.png");

            agujaTacometro = new TgcSprite();
            agujaTacometro.Texture = TgcTexture.createTexture(alumnoMediaFolder + "LucasArtsTribute\\Velocimetro\\agujaTacometro.png");

            //Ubicar Sprites
            indicador.Position = new Vector2(screenSize.Width - 260, screenSize.Height - 260);

            agujaVelocidad.Position = new Vector2(screenSize.Width - 260, screenSize.Height - 260);
            agujaTacometro.Position = new Vector2(screenSize.Width - 260, screenSize.Height - 260);

            //Cambia el centro de las agujas para la rotacion
            agujaVelocidad.RotationCenter = new Vector2(128, 128);
            agujaTacometro.RotationCenter = new Vector2(68, 183.5f);

            DxSprite = new Microsoft.DirectX.Direct3D.Sprite(GuiController.Instance.D3dDevice);


        }

        public void render()
        {
            GuiController.Instance.Drawer2D.beginDrawSprite();
            indicador.render();
            agujaVelocidad.render();
            agujaTacometro.render();
            GuiController.Instance.Drawer2D.endDrawSprite();

            //Renderizar textoCambio
            textCambio.render();
            textVelocity.render();
        }

        public void setVelocidad(float velocidad, float rpm)
        {
            textVelocity = new TgcText2d();
            textVelocity.Text = velocidad.ToString();
            textVelocity.Color = Color.Red;
            textVelocity.Align = TgcText2d.TextAlign.LEFT;
            textVelocity.Position = new Point(screenSize.Width - 130, screenSize.Height - 170);
            textVelocity.Size = new Size(75, 25);
            textVelocity.changeFont(new System.Drawing.Font("TimesNewRoman", 14, FontStyle.Bold));

            agujaVelocidad.Rotation = Geometry.DegreeToRadian(velocidad * 270 / 180);
            
            if (rpm == 0)
                rpm = 100;

            agujaTacometro.Rotation = Geometry.DegreeToRadian(rpm * 270 / 180);
        }

        public void setCambio(int s_gear)
        {
            
            //Crear textoCambio, especificando color, alineación, posición, tamaño y fuente.
            textCambio = new TgcText2d();
            textCambio.Text = s_gear.ToString();
            textCambio.Color = Color.Red;
            textCambio.Align = TgcText2d.TextAlign.LEFT;
            textCambio.Position = new Point(screenSize.Width - 140, screenSize.Height - 180);
            textCambio.Size = new Size(75, 25);
            textCambio.changeFont(new System.Drawing.Font("TimesNewRoman", 14, FontStyle.Bold));
            
        
        }

        public void dispose()
        {
            indicador.dispose();
            agujaTacometro.dispose();
            agujaVelocidad.dispose();
            textCambio.dispose();
            textVelocity.dispose();
        }
    }
    
}
