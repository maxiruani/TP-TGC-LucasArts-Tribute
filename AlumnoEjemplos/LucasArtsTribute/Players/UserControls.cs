using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX.DirectInput;

namespace AlumnoEjemplos.LucasArtsTribute.Players
{
    public interface IUserControls
    {
        Key Acelerate();
        Key Brake();
        Key GearUp();
        Key GearDown();
        Key Right();
        Key Left();
        Key Horn();
    }
    
    class UserControlsFactory
    {
        public static IUserControls Create(int numberOfPlayer)
        {
            if (numberOfPlayer == 1)
                return new UserControlsPlayerOne();
            if (numberOfPlayer == 2)
                return new UserControlsPlayerTwo();
            throw new Exception("Cantidad de Jugadores erronea.");
        }
    }

    /// <summary>
    /// Player One 
    ///             Acelerar W
    ///             Doblar A y D
    ///             Frenar S
    ///             Cambios B(Up) y V(Down)
    ///             Bocina G
    /// </summary>
    class UserControlsPlayerOne : IUserControls
    {
        public Key Acelerate()
        {
            return Key.W;
        }

        public Key Brake()
        {
            return Key.S;
        }

        public Key GearUp()
        {
            return Key.B;
        }

        public Key GearDown()
        {
            return Key.V;
        }

        public Key Right()
        {
            return Key.D;
        }

        public Key Left()
        {
            return Key.A;
        }
        public Key Horn()
        {
            return Key.G;
        }
    }


    /// <summary>
    /// Player One 
    ///             Acelerar Arriba
    ///             Doblar Derecha e Izq
    ///             Frenar Abajo
    ///             Cambios K(Up) y M(Down)
    ///             Bocina J
    /// </summary>
    class UserControlsPlayerTwo : IUserControls
    {
        public Key Acelerate()
        {
            return Key.UpArrow;
        }

        public Key Brake()
        {
            return Key.DownArrow;
        }

        public Key GearUp()
        {
            return Key.K;
        }

        public Key GearDown()
        {
            return Key.M;
        }

        public Key Right()
        {
            return Key.RightArrow;
        }

        public Key Left()
        {
            return Key.LeftArrow;
        }

        public Key Horn()
        {
            return Key.J;
        }
    }
}
