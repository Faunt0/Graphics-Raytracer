using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using static System.Formats.Asn1.AsnWriter;
using System.Net;
using OpenTK.Compute.OpenCL;

namespace Template
{
    class MyApplication
    {
        // member variables
        public Surface screen;
        public Surface debug;
        public Raytracer rt;
        public readonly bool isDebugging = true;
        public float a;
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
            a = 0;
            screen.Plot(1, 2, 0xff0000);
        }
        // tick: renders one frame
        public void Tick()
        {
            //debug.Clear(0);
            //debug.Print("DEBUG", 2, 2, 0xffffff);
            screen.Clear(0);
            rt.scene.lights[0].position = (rt.scene.lights[0].position.X + 0.05f * (float)Math.Cos(a), rt.scene.lights[0].position.Y, rt.scene.lights[0].position.Z);
            a += 0.05f;
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
            float fov = 90; // dit is niet een hele chille manier, moet nog anders
            float a = 1; // aspect ratio;

            // gebruik aspect ratio die je omrekent naar het verschil
            float z_dist = a / (float)(Math.Tan(fov / 2 * Math.PI / 180));


            Vector3 screen_center = position + z_dist * lookAtDir;
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
        public Vector3 color; // alpha? doubles? Maybe use Color4 object
        public Vector3 specular_color; // is niet nodig
        public float specularity;
        public Vector3 position;
        public Material material;

        public void SetSpecularity(Vector3 specular_color, float specularity)
        {
            this.specular_color = specular_color;
            this.specularity = specularity;
        }
        public class Material
        {
            // This needs to have multiple sub-classes which define properties
            public Tuple<float, float, float> spec_ref_mat;
        }
        public bool DoesOcclude(Ray ray, Scene scene)
        {
            // introduce an epsilon to combat shadow acne
            float epsilon = 0.000001f;

            Vector3 shadow_ray = ray.direction;
            Vector3 ins_point = ray.startPos;
            if (this is Sphere p)
            {
                // use math to calculate if there is at least one intersection
                Vector3 C = p.position;
                float a = Vector3.Dot(shadow_ray, shadow_ray);
                float b = -2f * Vector3.Dot(shadow_ray, C - ins_point);
                float c = Vector3.Dot(C - ins_point, C - ins_point) - p.radius * p.radius;

                float D = (float)Math.Pow(b, 2) - 4 * a * c;

                if (D >= 0)
                {
                    float t = Math.Min((-b + (float)Math.Sqrt(D)) / (2 * a), (-b - (float)Math.Sqrt(D)) / (2 * a));

                    //epsilon = 1.0f;
                    if (t < (scene.lights[0].position - ins_point).Length - epsilon && t > epsilon)
                    {
                        return true;
                    }
                }
            }
            else if (this is Plane pl)
            {
                if (Vector3.Dot(ray.direction, pl.normal) != 0)
                {
                    float t = Vector3.Dot((pl.position - ray.startPos), pl.normal) / Vector3.Dot(ray.direction, pl.normal);
                    if (t < (scene.lights[0].position - ins_point).Length - epsilon && t > epsilon)
                    {
                        return true;
                    }
                }
            }
            return false;
        }
        public class Sphere : Primitive
        {
            public float radius;
            public Sphere(Vector3 pos, float rad, Vector3 col)
            {
                this.position = pos;
                radius = rad;
                color = col;
            }
        }
        public class Plane : Primitive
        {
            public Vector3 normal;
            public float distance;
            public Plane(Vector3 norm, Vector3 p, Vector3 col)
            {
                normal = norm;
                this.position = p;
                color = col;
            }
        }
    }
    class Light
    {
        // point light
        public Vector3 position;
        public Vector3 intensity; // amount of light emitted by a point light source in one direction as a vector of the color it emits?
        public Light(Vector3 pos, Vector3 inte)
        {
            position = pos;
            intensity = inte;
        }
    }
    class Scene
    {
        public Primitive[] primitives;
        public Light[] lights;
        public int max_bounces;
        public List<(Ray, Vector3, int)> debugRays;
        public Scene()
        {
            max_bounces = 8;
            primitives = new Primitive[] { 
                new Primitive.Sphere((2, 0, 4), 0.5f, (1, 0, 0)), 
                new Primitive.Sphere((-2, 0, 4), 0.4f, (0, 1, 0)), 
                new Primitive.Sphere((0, 0, 4), .2f, (0, 0, 1)), 
                new Primitive.Plane((0f, 1, 0f), (0, -3, 4), (1, 1, 1))
            };
            for (int i = 0; i < 1; i++)
            {
                //primitives[i].SetSpecularity((0.8f, 0.8f, 0.8f), 3);
            }
            primitives[0].SetSpecularity((1, 1, 1), 3);
            //primitives[0].material.spec_ref_mat = new Tuple<float, float, float>(1, 0, 0);

            //lights = new Light[] { new Light(new(4, 1, 1), (1, 1, 1)) };
            lights = new Light[] { new Light(new(3, 1, 1), (20, 20, 20)) };

            debugRays = new List<(Ray, Vector3, int)>();
        }
        public Vector3 Trace(Ray ray, Scene s)
        {
            Vector3 color = new Vector3(0, 0, 0);
            Intersection ins = ray.GetIntersection(s);

            if (ins.nearestP != null)
            {
                if (ins.nearestP.specular_color == (1, 1, 1)) // does (1, 1, 1) mean its a pure specular
                {
                    Vector3 ins_point = ray.startPos + ins.distance * ray.direction;
                    Vector3 v = ins.distance * ray.direction;
                    Vector3 norm_reflected_dir = v - 2 * Vector3.Dot(v, ins.normal) * ins.normal;

                    Ray reflected_ray = new Ray(ins_point, norm_reflected_dir);
                    
                    if (ray.num_of_bounces < s.max_bounces)
                    {
                        reflected_ray.num_of_bounces = ray.num_of_bounces + 1;

                        // is dit wat ze bedoelen met material color?
                        Vector3 normal = new Vector3();
                        if (ins.nearestP is Primitive.Sphere)
                        {
                            Primitive.Sphere p = (Primitive.Sphere)ins.nearestP;
                            normal = Vector3.Normalize(ins_point - p.position); // get the normal at the intersection point
                        }
                        else if (ins.nearestP is Primitive.Plane p)
                        {
                            normal = p.normal;
                        }
                        
                        Vector3 ill_ray = new Vector3(s.lights[0].position - ins_point);
                        Vector3 r = Vector3.Normalize(ill_ray - 2 * (Vector3.Dot(ill_ray, normal)) * normal);
                        Vector3 materialColor = (float)Math.Pow(Math.Max(0, Vector3.Dot(ray.direction, r)), ins.nearestP.specularity) * ins.nearestP.specular_color;

                        //debugRays.Add((reflected_ray, ray.startPos, 0x00ff00));

                        color += materialColor + Trace(reflected_ray, s);
                    }
                }
                else
                {
                    if (ray.startPos != (0, 0, 0))
                    {
                        Console.WriteLine("hdf");
                    }

                    // calculate the phong diffuse shading etc

                    // calculate the point of the intersection
                    Vector3 ins_point = ray.startPos + ins.distance * ray.direction;

                    // illumination ray
                    Vector3 ill_ray = new Vector3(s.lights[0].position - ins_point);

                    // make the shadow ray
                    Vector3 shadow_ray = Vector3.Normalize(ill_ray);


                    bool occluded = false;
                    foreach (Primitive p in s.primitives)
                    {
                        if (p.DoesOcclude(ray, s)) occluded = true; break;
                    }


                    if (!occluded)
                    {
                        // if it is not occluded
                        Vector3 normal = new Vector3();
                        if (ins.nearestP is Primitive.Sphere)
                        {
                            Primitive.Sphere p = (Primitive.Sphere)ins.nearestP;
                            normal = Vector3.Normalize(ins_point - p.position); // get the normal at the intersection point
                        }
                        else if (ins.nearestP is Primitive.Plane p)
                        {
                            normal = p.normal;
                        }
                        Vector3 r = Vector3.Normalize(ill_ray - 2 * (Vector3.Dot(ill_ray, normal)) * normal);



                        //Vector3 sumparts =
                        //    Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color +
                        //    (float)Math.Pow(Math.Max(0, Vector3.Dot(ray.direction, r)), ins.nearestP.specularity) * ins.nearestP.specular_color;
                        Vector3 sumparts =
                            Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color +
                            (float)Math.Pow(Math.Max(0, Vector3.Dot(ray.direction, r)), ins.nearestP.specularity) * ins.nearestP.specular_color;

                        Vector3 L = EntryWiseMultiply(s.lights[0].intensity, (sumparts + ins.nearestP.color * (0.2f, 0.2f, 0.2f)) * (1 / (ill_ray.Length * ill_ray.Length)));

                        color += L;
                    }
                }
            }
            return Vector3.Clamp(color, (0, 0, 0), (1, 1, 1));
        }
        public Vector3 EntryWiseMultiply(Vector3 vec1, Vector3 vec2)
        {
            return (vec1.X * vec2.X, vec1.Y * vec2.Y, vec1.Z * vec2.Z);
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
        public int num_of_bounces;
        public Ray(Vector3 pos, Vector3 dir)
        {
            startPos = pos;
            direction = dir;
        }
        public Intersection GetIntersection(Scene s)
        {
            Intersection ins = new Intersection();

            foreach (Primitive prim in s.primitives)
            {
                // this must be possible in a more efficient and clean way.
                if (prim is Primitive.Sphere)
                {
                    Primitive.Sphere p = (Primitive.Sphere)prim;

                    // Use math
                    Vector3 C = p.position;
                    float a = Vector3.Dot(direction, direction); // wanneer is dit niet 1?
                    float b = -2f * Vector3.Dot(direction, C - startPos);
                    float c = Vector3.Dot(C - startPos, C - startPos) - p.radius * p.radius;

                    // calculate the discriminant
                    float D = (float)Math.Pow(b, 2) - 4 * a * c;

                    // only consider the case when there is an intersection
                    if (D >= 0)
                    {
                        // calculate the closest distance to the sphere
                        float t = Math.Min((-b + (float)Math.Sqrt(D)) / (2 * a), (-b - (float)Math.Sqrt(D)) / (2 * a));

                        // only store the shortest distance and thus the nearest Primitive
                        if (ins.distance > t || ins.nearestP == null)
                        {
                            ins.distance = t;
                            ins.nearestP = p;
                            ins.normal = Vector3.Normalize((startPos + ins.distance * direction) - ins.nearestP.position);
                        }
                    }
                }
                else if (prim is Primitive.Plane)
                {
                    Primitive.Plane p = (Primitive.Plane)prim;

                    if (Vector3.Dot(direction, p.normal) != 0)
                    {
                        float t = Vector3.Dot((p.position - startPos), p.normal) / Vector3.Dot(direction, p.normal);
                        if (t > 0 && (ins.distance > t || ins.nearestP == null))
                        {
                            ins.distance = t;
                            ins.nearestP = p;
                            ins.normal = p.normal;
                        }
                    }
                }
            }
            if (ins.distance < 0)
            {
                return new Intersection();
            }
            return ins;
        }
    }
    class Raytracer
    {
        public Scene scene;
        public Camera camera;
        public Surface screen;
        public List<Vector2> debugPoints;
        public Raytracer(Surface screen)
        {
            this.screen = screen;
            this.scene = new Scene();
            this.camera = new Camera();
            debugPoints = new List<Vector2>();
        }
        public void Render()
        {
            for (int i = 0; i < screen.pixels.Length; i++)
            {
                // get the corresponding x and y from the pixel index
                int x = i % screen.width;
                int y = i / screen.width;
                // get the corresponding point in the other coordinate system
                float a1 = (float)x / screen.width;
                float b1 = (float)y / screen.height;
                Vector3 u = camera.screenPlane[1] - camera.screenPlane[0];
                Vector3 v = camera.screenPlane[2] - camera.screenPlane[0];
                Vector3 point_on_screen = camera.screenPlane[0] + a1 * u + b1 * v;
                Vector3 pointScreen = point_on_screen;
                Vector3 ray_dir = pointScreen - camera.position;
                Vector3 norm_ray_dir = ray_dir / ray_dir.Length; //normalize
                Ray ray = new Ray(camera.position, norm_ray_dir);

                if (x == 500 && y == 501)
                {
                    continue;
                }
                Vector3 color = scene.Trace(ray, scene);


                screen.Plot(x, y, RGB2Int(color));




                //show the debug rays
                if (true && y == 500 && x % 20 == 0)
                {
                    Intersection ins = ray.GetIntersection(scene);
                    if (ins.nearestP != null)
                    {
                        Vector3 ins_point = ray.startPos + ins.distance * ray.direction;

                        debugPoints.Add((TX(ins_point.X), TY(ins_point.Z)));
                    }
                    else
                    {
                        float d = (5 - ray.startPos.Z) / ray.direction.Z;
                        Vector3 _ = ray.startPos + d * ray.direction;
                        int xval = Math.Max(TX(_.X), 0);
                        debugPoints.Add((TX(_.X), 0));
                    }
                }
            }
        }
        public void Debug()
        {
            // take only the pixels at y = 0 and plot them. also plot the primitives
            foreach (Primitive prim in scene.primitives)
            {
                if (prim is Primitive.Sphere)
                {
                    Primitive.Sphere p = (Primitive.Sphere)prim;
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
            }


            foreach (Vector2 point in debugPoints)
            {
                screen.Line(TX(camera.position.X), TY(camera.position.Z), (int)point.X, (int)point.Y, 0xffffff);
            }

            // secondary rays
            //if (scene.debugRays.Count > 0)
            //{
            //    (Ray, Vector3, int) debugRay = scene.debugRays[0];

            //    //float d = (5 - debugRay.Item1.startPos.Z) / debugRay.Item1.startPos.Z;
            //    Vector3 pointsa = debugRay.Item1.startPos + 1 * debugRay.Item1.direction;
            //    screen.Line(TX(debugRay.Item1.startPos.X), TY(debugRay.Item1.startPos.Z), TX(pointsa.X), TY(pointsa.Z), debugRay.Item3); 
            //}
            //for (int i = 0; i < scene.debugRays.Count; i += 40)
            //{
            //    (Ray, Vector3, int) debugRay = scene.debugRays[i];

            //    float d = (-5 - debugRay.Item1.startPos.X) / debugRay.Item1.startPos.X;
            //    Vector3 point = debugRay.Item1.startPos + d * debugRay.Item1.direction;
            //    screen.Line(TX(debugRay.Item2.X), TY(debugRay.Item2.Z), TX(point.X), TY(point.Z), debugRay.Item3);
            //}


            // plot the screen
            screen.Line(TX(camera.screenPlane[0].X), TY(camera.screenPlane[0].Z), TX(camera.screenPlane[1].X), TY(camera.screenPlane[1].Z), 0xffffff);

            // plot the light
            screen.Box(TX(scene.lights[0].position.X), TY(scene.lights[0].position.Z), TX(scene.lights[0].position.X) + 2, TY(scene.lights[0].position.Z) + 2, 0xffff00);

            // plot the camera
            screen.Box(TX(camera.position.X), TY(camera.position.Z), TX(camera.position.X) + 1, TY(camera.position.Z) + 1, 0xffffff);

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
            return (int)(rgb.X * 255) * 256 * 256 + (int)(rgb.Y * 255) * 256 + (int)(rgb.Z * 255);
            //return ((int)rgb.X << 16) + ((int)rgb.Y << 8) + ((int)rgb.Z);
            //return ((int)(rgb.X * 255) << 16) + ((int)(rgb.Y * 255) << 8) + ((int)(rgb.Z * 255));
            //return (int)(rgb.X * 255) * 256 * 256 + (int)(rgb.Y * 255) * 256 + (int)rgb.Z * 255;
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