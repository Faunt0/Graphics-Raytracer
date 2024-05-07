using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;

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
            rt.Render();
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
        public Vector3 rightDir;
        public Vector3[] screenPlane; // specified by the four corners

        public Camera()
        {
            // do something
            position = (0, 0, 0);
            lookAtDir = (0, 0, 1);
            upDir = (0, 1, 0);
            rightDir = (1, 0, 0);
            float fov = 2; // dit is niet een hele chille manier, moet nog anders
            float a = 1; // aspect ratio;

            Vector3 screen_center = position + fov * lookAtDir;
            // deze hoeken staan nu 2 verwijderd van de z: arbitrair hangt eigenlijk af van fov
            // fov is hoe dichtbij de camera staat van het scherm
            screenPlane = new Vector3[4] {
                screen_center + upDir - a * rightDir,
                screen_center + upDir + a * rightDir,
                screen_center - upDir - a * rightDir,
                screen_center - upDir + a * rightDir }; // weet niet of het laatste punt klopt.
        }
    }
    class Primitive
    {
        public Vector3 color; // alpha? doubles?

        public class Sphere : Primitive
        {
            public Vector3 position;
            public float radius;
            public Sphere(Vector3 pos, float rad, Vector3 col)
            {
                position = pos;
                radius = rad;
                color = col;
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
        public Primitive.Sphere[] primitives;
        public Light[] lights;
        public Scene()
        {
            primitives = new Primitive.Sphere[] { new Primitive.Sphere((2, 0, 4), 0.5f, (255, 0, 0)), new Primitive.Sphere((-2, 0, 4), 0.4f, (0, 255, 0)), new Primitive.Sphere((0, 0, 4), .2f, (0, 0, 255)) };
            //primitives = new Primitive.Sphere[] { new Primitive.Sphere((2, 0, 10), 0.5f, (255, 0, 0)), new Primitive.Sphere((0, 0, 10), .2f, (0, 0, 255)) };

            lights = new Light[] { new Light(new(4, 4, 0), 100f) };
        }
    }
    class Intersection
    {
        // stores the result of an intersection
        public float distance;
        public Primitive nearestP;
        public Vector3 normal; // in begin wss nog niet nodig
        //public Intersection(float distance, Primitive nearestP)
        //{
        //    this.distance = distance;
        //    this.nearestP = nearestP;
        //}
    }
    class Ray
    {
        public Vector3 startPos;
        public Vector3 direction; // is normalised
        public float intersection_dist;
        public Ray(Vector3 pos, Vector3 dir)
        {
            startPos = pos;
            direction = dir;
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
            for (int i = 0; i < screen.pixels.Length; i++)
            {
                // schrijf een functie die van een pixel een coordinaat maakt.
                // schiet rays
                int x = i % screen.width;
                int y = i / screen.width; //vgm werkt dit
                float a1 = x / screen.width;
                float b1 = y / screen.height;
                Vector3 u = camera.screenPlane[1] - camera.screenPlane[0];
                Vector3 v = camera.screenPlane[2] - camera.screenPlane[0];
                Vector3 point_on_screen = camera.screenPlane[0] + a1 * u + b1 * v;
                Vector3 pointScreen = point_on_screen;
                Vector3 ray_dir = pointScreen - camera.position;
                Vector3 norm_ray_dir = ray_dir / ray_dir.Length; //normalize
                Ray ray = new Ray(camera.position, norm_ray_dir);

                Intersection ins = new();

                foreach (Primitive.Sphere p in scene.primitives)
                {
                    Vector3 C = p.position;
                    float a = Vector3.Dot(norm_ray_dir, norm_ray_dir); // wanneer is dit niet 1?
                    float b = - 2f * Vector3.Dot(norm_ray_dir, C - camera.position);
                    float c = Vector3.Dot(C - camera.position, C - camera.position) - p.radius * p.radius;

                    float D = (float)Math.Pow(b, 2) - 4 * a * c;

                    if (D < 0)
                    {
                        // when the ray does not hit the sphere
                        continue;
                    }
                    else if (D == 0)
                    {
                        // when the ray is tangent to the sphere
                        // can a be zero in any way? When fov == 0 maybe?
                        float t = -b / (2 * a);
                        if (ins.distance > t)
                        {
                            ins.distance = t;
                            ins.nearestP = p;
                        }
                    }
                    else
                    {
                        float t = Math.Min((-b + (float)Math.Sqrt(D))/(2 * a), (-b - (float)Math.Sqrt(D)) / (2 * a));
                        if (ins.distance > t)
                        {
                            ins.distance = t;
                            ins.nearestP = p;
                        }
                    }
                }

                if (ins.nearestP != null)
                {
                    Vector3 col = new Vector3(ins.nearestP.color);
                    screen.Plot(x, y, RGB2Int(col));

                    //Vector3 ins_point = ray.startPos + ins.distance * ray.direction;

                    //// de hoek tussen de viewing ray en de illumination ray kan niet groter zijn dan 180 graden, anders is het licht achter het object vgm

                    //Vector3 ill_ray = scene.lights[0].position - ins_point;

                    //if (Math.Acos(Vector3.Dot(ill_ray, ray.direction) / (ill_ray.Length * ray.direction.Length)) < 90)
                    //{
                    //    Vector3 col = new Vector3(ins.nearestP.color);
                    //    screen.Plot(x, y, RGB2Int(col));
                    //}
                }




            }
        }
        public void Debug()
        {
            // take only the pixels at y = 0 and plot them. also plot the primitives
            foreach (Primitive.Sphere p in scene.primitives)
            {
                Vector2[] coords = new Vector2[100];
                for (int i = 0; i < 100; i++)
                {
                    float a = i * 2f * (float)Math.PI / 100f;
                    Vector2 coord = new Vector2((float)Math.Sin(a) * p.radius, (float)Math.Cos(a) * p.radius) + p.position.Xz;
                    coords[i] = coord;
                }
                for (int i = 0; i < 100 - 1; i++)
                {
                    screen.Line(TX(coords[i].X), TY(coords[i].Y), TX(coords[i + 1].X), TY(coords[i + 1].Y), RGB2Int(p.color));
                }
                screen.Line(TX(coords[0].X), TY(coords[0].Y), TX(coords[99].X), TY(coords[99].Y), RGB2Int(p.color));
            }

            //camera.position.Z -= 5;
            //for (int i = 0; i < 4; i++)
            //{
            //    camera.screenPlane[i].Z -= 5;
            //}

            // plot the screen
            screen.Line(TX(camera.screenPlane[0].X), TY(camera.screenPlane[0].Z), TX(camera.screenPlane[1].X), TY(camera.screenPlane[1].Z), 0xffffff);

            // plot the light
            screen.Box(TX(scene.lights[0].position.X), TY(scene.lights[0].position.Z), TX(scene.lights[0].position.X) + 2, TY(scene.lights[0].position.Z) + 2, 0xffff00);

            screen.Box(TX(camera.position.X), TY(camera.position.Z), TX(camera.position.X) + 1, TY(camera.position.Z) + 1, 0xffffff);

            //screen.Plot(TX(camera.position.X), TY(camera.position.Z), 0xffffff); 
            //screen.Plot(TX(camera.position.X) + 1, TY(camera.position.Z), 0xffffff); 
            //screen.Plot(TX(camera.position.X), TY(camera.position.Z) + 1, 0xffffff); 
            //screen.Plot(TX(camera.position.X) + 1, TY(camera.position.Z) + 1, 0xffffff); 
        }

        public int TX(float x)
        {
            float x1_s = x + 5;
            float x1_sc = x1_s * screen.width / 10;
            return (int)x1_sc;
        }
        public int TY(float y)
        {
            float y1_i = -y + 5;
            float y1_s = y1_i * screen.height / 10;
            return (int)y1_s;
        }

        // maak van de index van een pixel een coordinaat
        // moet dit een 3d coord zijn aangezien het scherm er ook nog moet staan?
        Vector3 PX(int i)
        {
            float y = -(10 * (i % screen.width) / screen.height - 5);
            float x = i / screen.width * 10 - 5;
            float z = 0;
            return new Vector3(x, y, z);
        }
        int RGB2Int(Vector3 rgb)
        {
            return (int)rgb.X * 256 * 256 + (int)rgb.Y * 256 + (int)rgb.Z;
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