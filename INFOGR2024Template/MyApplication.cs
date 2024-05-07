using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;

namespace Template
{
    class MyApplication
    {
        // member variables
        public Surface screen;
        public Surface debug;
        public Raytracer rt;
        public readonly bool isDebugging = true;
        // constructor
        public MyApplication(Surface screen, Surface debug)
        {
            this.screen = screen;
            this.debug = debug;
        }
        // initialize
        public void Init()
        {
            rt = new Raytracer(screen);
            rt.Debug();
            screen.Plot(1, 2, 0xff0000);
        }
        // tick: renders one frame
        public void Tick()
        {
            //debug.Clear(0);
            //debug.Print("DEBUG", 2, 2, 0xffffff);
            screen.Clear(0);
            rt.Debug();
            //screen.pixels = rt.screen.pixels;
            //screen.Print("hello world", 2, 2, 0xffffff);
            //screen.Line(2, 20, 160, 20, 0xff0000);
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
        public Vector3 normal; // in begin wss nog niet nodig
    }
    class Ray
    {
        public Vector3 position;
        public Vector3 direction; // is normalised
        public float int_dist;
        public Ray()
        {

        }
    }
    class Raytracer
    {
        public Scene scene;
        public Camera camera;
        public Surface screen;
        public Raytracer(Surface screen)
        {
            this.screen = screen;
            this.scene = new Scene();
            this.camera = new Camera();
        }
        public void Render()
        {
            // debug
            Console.WriteLine(screen.pixels.Length);
            for (int i = 0; i < screen.pixels.Length; i++)
            {
                // schrijf een functie die van een pixel een coordinaat maakt.
                
            }
        }
        public void Debug()
        {
            // take only the pixels at y = 0 and plot them. also plot the primitives
            foreach (Primitive p in scene.primitives)
            {
                
            }

            // en voor elke ray

            screen.Print("hello world", 2, 2, 0xffffff);
            screen.Plot(TX(camera.position.X), TY(camera.position.Z), 0xffffff); // moet hier TY gebruikt worden?
            screen.Plot(TX(camera.position.X) + 1, TY(camera.position.Z), 0xffffff); // moet hier TY gebruikt worden?
            screen.Plot(TX(camera.position.X), TY(camera.position.Z) + 1, 0xffffff); // moet hier TY gebruikt worden?
            screen.Plot(TX(camera.position.X) + 1, TY(camera.position.Z) + 1, 0xffffff); // moet hier TY gebruikt worden?
        }

        public int TX(float x)
        {
            float x1_s = x + 5;
            float x1_sc = x1_s * screen.width / 10 * (screen.height / screen.width);
            return (int)x1_sc;
        }
        public int TY(float y)
        {
            float y1_i = -y + 5;
            float y1_s = y1_i * (screen.height / 10 * (screen.height / screen.width));
            return (int)y1_s;
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