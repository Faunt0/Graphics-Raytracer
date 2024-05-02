using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;

namespace Template
{
    class MyApplication
    {
        // member variables
        public Surface screen;
        public Surface debug;
        // constructor
        public MyApplication(Surface screen, Surface debug)
        {
            this.screen = screen;
            this.debug = debug; 
        }
        // initialize
        public void Init()
        {
            //screen.Plot()
        }
        // tick: renders one frame
        public void Tick()
        {
            //debug.Clear(0);
            //debug.Print("DEBUG", 2, 2, 0xffffff);
            screen.Clear(0);
            screen.Print("hello world", 2, 2, 0xffffff);
            screen.Line(2, 20, 160, 20, 0xff0000);
        }

        public int TX(float x)
        {
            float x1_s = x + 5;
            float x1_sc = x1_s * screen.width / 4;
            return (int)x1_sc;
        }
        public int TY(float y)
        {
            float y1_i = -y + 5;
            float y1_s = y1_i * (screen.height / 2);
            return (int)y1_s;
        }
    }
    class Camera
    {
        public Vector3 position;
        public Vector3 lookAtDir;
        public Vector3 upDir;
        public Vector3[] screenPlane; // specified by the four corners

        public Camera()
        {
            // do something
            position = (0, 0, 0);
            lookAtDir = (0, 0, 1);
            upDir = (0, 1, 0);
        }
    }
    class Primitive
    {
        public Vector3 color; // alpha? doubles?

        public class Sphere : Primitive
        {
            public Vector3 position;
            public float radius;
            public Sphere(Vector3 pos, float rad)
            {
                position = pos;
                radius = rad;
            }
        }
        public class Plane : Primitive
        {
            public Vector3 normal;
            public Vector3 distance;
            public Plane(Vector3 norm, Vector3 dist)
            {
                normal = norm;
                distance = dist;
            }
        }
    }
    class Light
    {
        // point light
        public Vector3 position;
        public float intensity; // how
        public Light(Vector3 pos, float inte)
        {
            position = pos;
            intensity = inte;
        }
    }
    class Scene
    {
        public Primitive[] primitives;
        public Light[] lights;
        public Scene()
        {
            primitives = new Primitive[] { new Primitive.Sphere((2, 0, 0), 1.0f), new Primitive.Sphere((4, 0, 0), 1.0f), new Primitive.Sphere((0, 0, 0), 1.0f)};
        }
    }
    class Intersection
    {
        // stores the result of an intersection
        public float distance;
        public Primitive nearestP;
        public Vector3 normal;
    }
    class Raytracer
    {
        public Scene scene;
        public Camera camera;
        public Surface screen;
        public Raytracer(Surface screen)
        {
            //display.pixels
            this.screen = screen;

        }
        public void Render()
        {
            // debug
            screen.pixels = new int[] { };
        }
        public void Debug()
        {
            foreach (Primitive p in scene.primitives)
            {
                
            }
            // en voor elke ray
        }
    }
    class Application
    { 
        // handles keyboard and mouse input
        public Application()
        {
            //Raytracer rt = new Raytracer();
            //rt.Render();
        }
    }
}