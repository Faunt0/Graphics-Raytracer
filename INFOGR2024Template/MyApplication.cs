using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.GraphicsLibraryFramework;
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
        public bool debugb = false;
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
            screen.Clear(0);
            a += 0.05f;
            if (debugb)
            {
                //rt.Render();
                rt.Debug();
            }
            else
            {
                rt.Render();
            };
        }
    }
    class Camera
    {
        public Vector3 position;
        public Vector3 lookAtDir;
        public Vector3 upDir;
        public Vector3 rightDir;
        public Vector3[] screenPlane; // specified by the four corners
        float fov = 90; // dit is niet een hele chille manier, moet nog anders
        float a; // aspect ratio;
        float d;
        public Camera(float aspectratio)
        {
            // do something
            a = aspectratio;
            position = (0, 0, 0);
            lookAtDir = (0, 0, 1);
            upDir = (0, 1, 0);
            rightDir = (1, 0, 0);
            d = rightDir.X * a / (float)(Math.Tan((fov / 2) * Math.PI / 180.0));
            Vector3 screen_center = position + d * lookAtDir;
            Console.WriteLine((Math.Tan((fov / 2) * Math.PI / 180.0)));
            // fov is hoe dichtbij de camera staat van het scherm
            screenPlane = new Vector3[4] {
                screen_center + upDir - a * rightDir,
                screen_center + upDir + a * rightDir,
                screen_center - upDir - a * rightDir,
                screen_center - upDir + a * rightDir };
        }

        // Move is a public method for updating the camera's position with user input
        // it also calls Refresh, so the screen plane follows the new camera position
        // https://opentk.net/learn/chapter1/9-camera.html?tabs=input-opentk4%2Cdelta-time-input-opentk4%2Ccursor-mode-opentk4%2Cmouse-move-opentk4%2Cscroll-opentk4
        public void Move(string dir)
        {
            // amount to move the camera
            float speed = 0.5f;

            switch (dir)
            {
                case "up":
                    this.position += speed * upDir;
                    break;
                case "down":
                    this.position -= speed * upDir;
                    break;
                case "backward":
                    this.position -= speed * lookAtDir;
                    break;
                case "forward":
                    this.position += speed * lookAtDir;
                    break;
                case "left":
                    this.position -= speed * rightDir;
                    break;
                case "right":
                    this.position += speed * rightDir;
                    break;
            }

            Refresh();
        }

        public void CameraAngle(string dir)
        {
            float degrees = 180/(float)Math.PI * 0.005f;
            switch (dir)
            {
                case "up":
                    lookAtDir = Pitch(-degrees, lookAtDir);
                    upDir = Pitch(-degrees, upDir);
                    break;
                case "down":
                    lookAtDir = Pitch(degrees, lookAtDir);
                    upDir = Pitch(degrees, upDir);
                    break;
                case "left":
                    lookAtDir = Yaw(degrees, lookAtDir);
                    rightDir = Yaw(degrees, rightDir);
                    break;
                case "right":
                    lookAtDir = Yaw(-degrees, lookAtDir);
                    rightDir = Yaw(-degrees, rightDir);
                    break;
            }
            Refresh();
        }
        public Vector3 Pitch(float beta, Vector3 vec)
        {
            Matrix3 pitch = Matrix3.CreateRotationX(beta);
            return pitch * vec;
        }
        public Vector3 Yaw(float beta, Vector3 vec)
        {
            Matrix3 yaw = Matrix3.CreateRotationY(beta);
            return yaw * vec;
        }


        // Refresh ensures that the screen plane follows the camera
        public void Refresh()
        {
            Vector3 screen_center = position + d * lookAtDir;
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

        public void SetSpecularity(float specularity)
        {
            this.specularity = specularity;
        }
        public void SetMaterial(float specular, float reflect, float mat, bool isMetal)
        {
            material = new Material(specular, reflect, mat, isMetal);
        }
        public bool DoesOcclude(Ray ray, Scene scene)
        {
            // introduce an epsilon to combat shadow acne
            float epsilon = 0.0001f;

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
        public class Material
        {
            // Subclasses for different materials?
            public float spec_index;
            public float ref_index;
            public float mat_index;
            public bool isMetal;
            public Material(float spec_index, float ref_index, float mat_index, bool isMetal)
            {
                this.spec_index = spec_index;
                this.ref_index = ref_index;
                this.mat_index = mat_index;
                this.isMetal = isMetal;
            }
        }
        public class Sphere : Primitive
        {
            public float radius;
            public Sphere(Vector3 pos, float rad, Vector3 col)
            {
                this.position = pos;
                this.radius = rad;
                this.color = col;
            }
        }
        public class Plane : Primitive
        {
            public Vector3 normal;
            public float distance;
            public Vector3 uvec;
            public Vector3 vvec;
            public Plane(Vector3 norm, Vector3 p, Vector3 col)
            {
                this.normal = norm;
                this.position = p;
                this.color = col;

                // calculate the vectors for u and v for textures
                uvec = Vector3.Normalize(Vector3.Cross(normal, new Vector3(1, 0, 0)));
                if (uvec == new Vector3(0, 0, 0))
                {
                    uvec = Vector3.Normalize(Vector3.Cross(normal, new Vector3(0, 0, 1)));
                }
                vvec = Vector3.Normalize(Vector3.Cross(normal, uvec));
            }
        }
    }
    class Light
    {
        // point light
        public Vector3 position;
        public Vector3 intensity;
        public Light(Vector3 pos, Vector3 inte)
        {
            this.position = pos;
            this.intensity = inte;
        }
    }
    class Scene
    {
        public Primitive[] primitives;
        public Light[] lights;
        public int max_bounces;
        public List<(Ray, int)> debugRays;
        public List<(Vector2, Vector2, int)> debugPoints;
        public bool isDebugRay = false;
        public Scene()
        {
            max_bounces = 8;
            primitives = new Primitive[] {
                new Primitive.Sphere((2, 0, 4), 0.5f, (1, 0, 0)),
                new Primitive.Sphere((-2, 0, 4), 0.4f, (0, 1, 0)),
                new Primitive.Sphere((0, 0, 4), .2f, (0, 0, 1)),
                new Primitive.Sphere((0, 2, 2), .6f, (158/255, 163/255, 168/255)),
                new Primitive.Plane((0f, 1, 0f), (0, -1, 3), (1, 1, 1))
            };
            primitives[0].SetSpecularity(3);
            primitives[0].SetMaterial(0, 0.5f, 0.5f, false);

            primitives[1].SetSpecularity(3);
            primitives[1].SetMaterial(0.5f, 0, .5f, false);

            primitives[2].SetSpecularity(4);
            primitives[2].SetMaterial(0.5f, 0, 0.5f, false);

            primitives[3].SetSpecularity(2);
            primitives[3].SetMaterial(0.5f, 0.5f, 0, true);

            primitives[4].SetSpecularity(4);
            primitives[4].SetMaterial(0.5f, 0.5f, 0, false);


            lights = new Light[] { new Light(new(3, 1, 1), (10, 10, 10)), new Light((-3, 1, 1), (10, 10, 10)), new Light((0, 3, 0), (20, 0, 0)) };

            debugRays = new List<(Ray, int)>();
        }
        public Vector3 Trace(Ray ray, Scene s)
        {
            Vector3 color = new Vector3(0, 0, 0);
            Intersection ins = ray.GetIntersection(s);

            if (ins.nearestP != null && (ray.parent_ray == null || ray.hit_prim != ray.parent_ray.hit_prim))
            {
                if (ins.nearestP.material.ref_index > 0)
                {
                    Vector3 ins_point = ins.point;
                    Vector3 norm_reflected_dir = ray.direction - 2 * Vector3.Dot(ray.direction, ins.normal) * ins.normal;

                    Ray reflected_ray = new Ray(ins_point, norm_reflected_dir);
                    reflected_ray.parent_ray = ray;
                    reflected_ray.isSecondaryRay = true;
                    
                    if (ray.num_of_bounces < s.max_bounces)
                    {
                        reflected_ray.num_of_bounces = ray.num_of_bounces + 1;

                        if (isDebugRay)
                        {
                            debugRays.Add((reflected_ray, 0x00ff00));
                        }

                        color += CalculateColor(ins.nearestP, ray, ins) + ins.nearestP.material.ref_index * Trace(reflected_ray, s);
                    }
                }
                else
                {
                    color += CalculateColor(ins.nearestP, ray, ins);
                    isDebugRay = false;
                }
            }
            return color;
        }
        public Vector3 CalculateColor(Primitive prim, Ray ray, Intersection ins)
        {
            // calculate the point of the intersection
            Vector3 ins_point = ray.startPos + ins.distance * ray.direction;
            Vector3 L = new Vector3(0, 0, 0);

            // illumination ray
            foreach (Light l in lights)
            {
                Vector3 ill_ray = new Vector3(l.position - ins_point);

                if (isDebugRay)
                {
                    debugPoints.Add((ins_point.Xz, l.position.Xz, 0xffff00));
                }
                // make the shadow ray
                Vector3 shadow_ray = Vector3.Normalize(ill_ray);
                Ray sray = new Ray(shadow_ray, ins_point);

                bool occluded = false;
                Primitive occludedBy = new Primitive();
                if (!ray.isSecondaryRay)
                {
                    foreach (Primitive p in primitives)
                    {
                        if (p.DoesOcclude(sray, this))
                        {
                            occluded = true; occludedBy = p;
                        }
                        if (occludedBy == prim)
                        {
                            occluded = false;
                        }
                    }
                }


                if (!occluded)
                {
                    // if it is not occluded
                    Vector3 normal = new Vector3();
                    if (ins.nearestP is Primitive.Sphere)
                    {
                        Primitive.Sphere p2 = (Primitive.Sphere)ins.nearestP;
                        normal = Vector3.Normalize(ins_point - p2.position); // get the normal at the intersection point
                    }
                    else if (ins.nearestP is Primitive.Plane)
                    {
                        Primitive.Plane p1 = (Primitive.Plane)ins.nearestP;
                        normal = p1.normal;
                    }
                    Vector3 r = Vector3.Normalize(ill_ray - 2 * (Vector3.Dot(ill_ray, normal)) * normal);

                    Vector3 sumparts =
                        ins.nearestP.material.mat_index * Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color +
                        ins.nearestP.material.spec_index * Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color +
                        ins.nearestP.material.spec_index * (float)Math.Pow(Math.Max(0, Vector3.Dot(ray.direction, r)), ins.nearestP.specularity) * ins.nearestP.specular_color;

                    // deal with metals
                    float temp = ins.nearestP.material.spec_index * (float)Math.Pow(Math.Max(0, Vector3.Dot(ray.direction, r)), ins.nearestP.specularity);
                    if (ins.nearestP.material.isMetal)
                    {
                        sumparts += temp * ins.nearestP.color;
                    }
                    else
                    {
                        sumparts += temp * new Vector3(1, 1, 1);
                    }
                    L += EntryWiseMultiply(l.intensity, sumparts * (1 / (ill_ray.Length * ill_ray.Length)));
                }
            }
            // Add ambient color
            L += (1 - ins.nearestP.material.ref_index) * ins.nearestP.color * (0.2f, 0.2f, 0.2f);
            return L;
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
        public Vector3 normal;
        public Vector3 point;
    }
    class Ray
    {
        public Vector3 startPos;
        public Vector3 direction; // is normalised
        public Primitive? hit_prim;
        public Ray? parent_ray;
        public bool isSecondaryRay;
        public int num_of_bounces;
        public Ray(Vector3 pos, Vector3 dir)
        {
            startPos = pos;
            direction = dir;
        }
        public Intersection GetIntersection(Scene s)
        {
            float epsilon = 0.00001f;
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
                        if ((ins.distance - epsilon > t && t > epsilon) || (ins.nearestP == null && t > epsilon))
                        {
                            ins.point = startPos + t * direction;
                            ins.distance = Math.Abs(t);
                            ins.nearestP = p;
                            ins.normal = Vector3.Normalize((startPos + ins.distance * direction) - ins.nearestP.position);

                            hit_prim = p;
                        }
                    }
                }
                else if (prim is Primitive.Plane)
                {
                    Primitive.Plane p = (Primitive.Plane)prim;

                    if (Vector3.Dot(direction, p.normal) != 0)
                    {
                        float t = Vector3.Dot((p.position - startPos), p.normal) / Vector3.Dot(direction, p.normal);
                        if ((t > epsilon && ins.distance - epsilon > t) || (ins.nearestP == null && t > epsilon))
                        {
                            ins.point = startPos + t * direction;
                            ins.distance = Math.Abs(t);
                            ins.nearestP = p;
                            ins.normal = p.normal;

                            hit_prim = p;
                        }
                    }
                }
            }
            return ins;
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
            this.camera = new Camera((float)screen.width/(float)screen.height);
            scene.debugPoints = new List<(Vector2, Vector2, int)>();
        }
        public void Render()
        {
            scene.debugRays.Clear();
            scene.debugPoints.Clear();
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


                // make sure the values don't overflow
                Vector3 color = Vector3.Clamp(scene.Trace(ray, scene), (0,0,0), (1,1,1));


                screen.Plot(x, y, RGB2Int(color));

                //show the debug rays
                if (y == (float)screen.height/2 && x % 15 == 0)
                {
                    scene.isDebugRay = true;
                    Intersection ins = ray.GetIntersection(scene);


                    // debug rays vanaf de camera
                    if (ins.nearestP != null)
                    {
                        Vector3 ins_point = ray.startPos + ins.distance * ray.direction;
                        scene.debugPoints.Add(
                            (
                            new(TX(ins_point.X), TY(ins_point.Z)), 
                            new(TX(ray.startPos.X), TY(ray.startPos.Z)),
                            0xffffff));
                    }
                    else
                    {
                        float d = (5 - ray.startPos.Z) / ray.direction.Z;
                        Vector3 _ = ray.startPos + d * ray.direction;
                        scene.debugPoints.Add((new(TX(_.X), 0),
                            new(TX(ray.startPos.X), TY(ray.startPos.Z)), 0xffffff));
                    }
                }
                else
                {
                    scene.isDebugRay = false;
                }
            }
        }
        public void Debug()
        {
            // take only the pixels at y = 0 and plot them. also plot the primitives
            scene.debugRays.Clear();
            scene.debugPoints.Clear();
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

                Vector3 color = Vector3.Clamp(scene.Trace(ray, scene), (0, 0, 0), (1, 1, 1));

                //show the debug rays
                if (y == (float)screen.height / 2 && x % 15 == 0)
                {
                    scene.isDebugRay = true;
                    Intersection ins = ray.GetIntersection(scene);


                    // debug rays vanaf de camera
                    if (ins.nearestP != null)
                    {
                        Vector3 ins_point = ray.startPos + ins.distance * ray.direction;
                        scene.debugPoints.Add((new(TX(ins_point.X), TY(ins_point.Z)), new(TX(ray.startPos.X), TY(ray.startPos.Z)), 0xffffff));
                    }
                    else
                    {
                        float d = (5 - ray.startPos.Z) / ray.direction.Z;
                        Vector3 _ = ray.startPos + d * ray.direction;
                        scene.debugPoints.Add((new(TX(_.X), 0), new(TX(ray.startPos.X), TY(ray.startPos.Z)), 0xffffff));
                    }
                }
                else
                {
                    scene.isDebugRay = false;
                }
            }


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


            foreach ((Vector2, Vector2, int) dbgp in scene.debugPoints)
            {
                screen.Line((int)dbgp.Item1.X, (int)dbgp.Item1.Y, (int)dbgp.Item2.X, (int)dbgp.Item2.Y, dbgp.Item3);
            }

            // secondary rays
            for (int i = 0; i < scene.debugRays.Count; i++)
            {
                (Ray, int) debugRay = scene.debugRays[i];
                if (debugRay.Item1.startPos.Y == 0)
                {
                    Intersection ins = debugRay.Item1.GetIntersection(scene);
                    Vector3 pointsa = new Vector3();
                    if (ins.nearestP != null)
                    {
                        pointsa = ins.point;
                    }
                    else
                    {
                        pointsa = debugRay.Item1.startPos + 4 * debugRay.Item1.direction;
                    }
                    screen.Line(TX(debugRay.Item1.startPos.X), TY(debugRay.Item1.startPos.Z), TX(pointsa.X), TY(pointsa.Z), debugRay.Item2);
                }
            }


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
            float y1_i = -y + (5f*(screen.width/screen.height));
            float y1_s = y1_i * screen.height / (10f* (screen.width / screen.height));
            return (int)y1_s;
        }
        int RGB2Int(Vector3 rgb)
        {
            return (int)(rgb.X * 255) * 256 * 256 + (int)(rgb.Y * 255) * 256 + (int)(rgb.Z * 255);
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