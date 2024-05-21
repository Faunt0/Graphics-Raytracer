using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.GraphicsLibraryFramework;
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
        public bool debugb = false;
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
            //rt.scene.lights[0].position = (rt.scene.lights[0].position.X + 0.05f * (float)Math.Cos(a), rt.scene.lights[0].position.Y, rt.scene.lights[0].position.Z + 0.05f * (float)Math.Cos(a));
            a += 0.05f;
            if (debugb)
            {
                rt.Debug();
            }
            else
            {
                rt.Render();
            };
            
            
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
        float fov = 90; // dit is niet een hele chille manier, moet nog anders
        float a = 1; // aspect ratio;
        public Camera()
        {
            // do something
            position = (0, 0, 0);
            lookAtDir = (0, 0, 1);
            upDir = (0, 1, 0);
            rightDir = (1, 0, 0);
            float d = rightDir.X * a / (float)(Math.Tan((fov/2)* Math.PI / 180.0));
            Vector3 screen_center = position + d * lookAtDir;
            Console.WriteLine((Math.Tan((fov / 2) * Math.PI / 180.0)));
            // deze hoeken staan nu 2 verwijderd van de z: arbitrair hangt eigenlijk af van fov
            // fov is hoe dichtbij de camera staat van het scherm
            screenPlane = new Vector3[4] {
                screen_center + upDir - a * rightDir,
                screen_center + upDir + a * rightDir,
                screen_center - upDir - a * rightDir,
                screen_center - upDir + a * rightDir }; // weet niet of het laatste punt klopt.
        }

        public void Move(string dir)
        {
            float speed = 0.5f;

            switch (dir)
            {
                case "up":
                    this.position.Y += speed;
                    break;
                case "down":
                    this.position.Y -= speed;
                    break;
                case "backwards":
                    this.position.Z += speed;
                    break;
                case "forwards":
                    this.position.Z -= speed;
                    break;
                case "left":
                    this.position.X += speed;
                    break;
                case "right":
                    this.position.X -= speed;
                    break;
            }

            Refresh();
        }

        public void Refresh()
        {
            
            float d = rightDir.X * a / (float)(Math.Tan((fov / 2) * Math.PI / 180.0));
            Vector3 screen_center = position + d * lookAtDir;
            Console.WriteLine((Math.Tan((fov / 2) * Math.PI / 180.0)));
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
        public Vector3 specular_color;
        public float specularity;

        public void SetSpecularity(Vector3 specular_color, float specularity)
        {
            this.specular_color = specular_color;
            this.specularity = specularity;
        }

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
            public Vector3 p0;
            public float distance;
            public Plane(Vector3 norm, Vector3 p, Vector3 col)
            {
                normal = norm;
                p0 = p;
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
        public Scene()
        {
            primitives = new Primitive[] { 
                new Primitive.Sphere((2, 0, 4), 0.5f, (1, 0, 0)), 
                new Primitive.Sphere((-2, 0, 4), 0.4f, (0, 1, 0)), 
                new Primitive.Sphere((0, 0, 4), .2f, (0, 0, 1)), 
                new Primitive.Plane((0, 1, 0f), (0, -3, 4), (1, 1, 1))
            };
            for (int i = 0; i < 3; i++)
            {
                primitives[i].SetSpecularity((1f, 1f, 1f), 3);
            }
            //primitives[0].SetSpecularity((1, 1, 1), 10);

            //lights = new Light[] { new Light(new(4, 1, 1), (1, 1, 1)) };
            lights = new Light[] { new Light(new(3, 1, 1), (20, 20, 20)) };
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
                // get the corresponding x and y from the pixel index
                int x = i % screen.width;
                int y = i / screen.width; //vgm werkt dit
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


                Intersection ins = new();

                // calculate the intersections
                foreach (Primitive prim in scene.primitives)
                {
                    // this must be possible in a more efficient and clean way.
                    if (prim is Primitive.Sphere)
                    {
                        Primitive.Sphere p = (Primitive.Sphere)prim;

                        // Use math
                        Vector3 C = p.position;
                        float a = Vector3.Dot(norm_ray_dir, norm_ray_dir); // wanneer is dit niet 1?
                        float b = -2f * Vector3.Dot(norm_ray_dir, C - camera.position);
                        float c = Vector3.Dot(C - camera.position, C - camera.position) - p.radius * p.radius;

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
                            }
                        }
                    }
                    if (prim is Primitive.Plane)
                    {
                        Primitive.Plane p = (Primitive.Plane)prim;
                        
                        if (Vector3.Dot(norm_ray_dir, p.normal) != 0)
                        {
                            float t = Vector3.Dot((p.p0 - camera.position), p.normal) / Vector3.Dot(norm_ray_dir, p.normal);
                            if (t > 0 && (ins.distance > t || ins.nearestP == null))
                            {
                                ins.distance = t;
                                ins.nearestP = p;
                            }
                        }
                    }
                }


                // get the color of the pixel
                if (ins.nearestP != null)
                {
                    // start black?
                    Vector3 col = new Vector3(0, 0, 0);

                    // calculate the point of the intersection
                    Vector3 ins_point = ray.startPos + ins.distance * ray.direction;

                    // illumination ray
                    Vector3 ill_ray = new Vector3(scene.lights[0].position - ins_point);

                    // make the shadow ray
                    Vector3 shadow_ray = Vector3.Normalize(ill_ray);

                    // calculate for every primitive if this occludes the shadow ray.
                    bool occluded = false;
                    foreach (Primitive prim in scene.primitives)
                    {
                        // change to a more efficient approach by making functions per primitive
                        if (prim is Primitive.Sphere)
                        {
                            Primitive.Sphere p = (Primitive.Sphere)prim;
                            // use math to calculate if there is at least one intersection
                            Vector3 C = p.position;
                            float a = Vector3.Dot(shadow_ray, shadow_ray);
                            float b = -2f * Vector3.Dot(shadow_ray, C - ins_point);
                            float c = Vector3.Dot(C - ins_point, C - ins_point) - p.radius * p.radius;

                            float D = (float)Math.Pow(b, 2) - 4 * a * c;

                            if (D >= 0)
                            { 
                                float t = Math.Min((-b + (float)Math.Sqrt(D)) / (2 * a), (-b - (float)Math.Sqrt(D)) / (2 * a));

                                // introduce an epsilon to combat shadow acne
                                float epsilon = 0.01f;
                                //epsilon = 1.0f;
                                if (t < (scene.lights[0].position - ins_point).Length - epsilon && t > epsilon)
                                {
                                    occluded = true;
                                    break;
                                }
                            }
                        }
                        else if (prim is Primitive.Plane)
                        {
                            Primitive.Plane p = (Primitive.Plane)prim;
                        }
                    }


                    // make colours
                    if (!occluded)
                    {
                        // introduce diffuse materials
                        Vector3 normal = new Vector3();
                        if (ins.nearestP is Primitive.Sphere)
                        {
                            Primitive.Sphere p = (Primitive.Sphere)ins.nearestP;
                            normal = Vector3.Normalize(ins_point - p.position); // get the normal at the intersection point
                        }
                        else if (ins.nearestP is Primitive.Plane)
                        {
                            Primitive.Plane p = (Primitive.Plane)ins.nearestP;
                            normal = p.normal;
                        }
                        Vector3 r = Vector3.Normalize(ill_ray - 2 * (Vector3.Dot(ill_ray, normal)) * normal);




                        //Vector3 L_part1 = (1 / (shadow_ray.Length * shadow_ray.Length)) * 
                        //    Math.Max(0, Vector3.Dot(normal, shadow_ray) / (normal.Length * norm_ray_dir.Length)) *
                        //    ins.nearestP.color;

                        //Vector3 L_part2 = (1 / (shadow_ray.Length * shadow_ray.Length)) * 
                        //    (float)Math.Pow(Math.Max(0, Vector3.Dot(norm_ray_dir, r)), ins.nearestP.specularity) *
                        //    ins.nearestP.specular_color;
                        //col = EntryWiseMultiply(scene.lights[0].intensity, L_part1);

                        //col += EntryWiseMultiply(scene.lights[0].intensity, L_part2);



                        // dit werk niet optimaal, zitten wat rare artifacts in. Overflow namelijk
                        Vector3 sumparts =
                            Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color + 
                            (float)Math.Pow(Math.Max(0, Vector3.Dot(norm_ray_dir, r)), ins.nearestP.specularity) * ins.nearestP.specular_color;
                        Vector3 L = EntryWiseMultiply(scene.lights[0].intensity, sumparts * (1 / (ill_ray.Length * ill_ray.Length))) + ins.nearestP.color * (0.2f, 0.2f, 0.2f);

                        //col = (Math.Min(1, L.X), Math.Min(1, L.Y), Math.Min(1, L.Z));
                        col = Vector3.Clamp(L, (0, 0, 0), (1, 1, 1));



                        // THIS WORKS
                        //Vector3 L = scene.lights[0].intensity * (1 / (shadow_ray.Length * shadow_ray.Length)) * Math.Max(0, (float)Math.Cos(Vector3.CalculateAngle(normal, shadow_ray)));
                        //float Lx = L.X * ins.nearestP.color.X;
                        //float Ly = L.Y * ins.nearestP.color.Y;
                        //float Lz = L.Z * ins.nearestP.color.Z;




                        //(float)Math.Cos(Vector3.CalculateAngle(normal, shadow_ray));
                        //float Lx = scene.lights[0].intensity.X * (Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color.X);
                        //float Ly = scene.lights[0].intensity.Y * (Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color.Y);
                        //float Lz = scene.lights[0].intensity.Z * (Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color.Z);
                        //float Lx = scene.lights[0].intensity.X * (1 / (ill_ray.Length * ill_ray.Length)) * (Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color.X);
                        //float Ly = scene.lights[0].intensity.Y * (1 / (ill_ray.Length * ill_ray.Length)) * (Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color.Y);
                        //float Lz = scene.lights[0].intensity.Z * (1 / (ill_ray.Length * ill_ray.Length)) * (Math.Max(0, Vector3.Dot(normal, shadow_ray)) * ins.nearestP.color.Z);

                        //col = (Lx, Ly, Lz);

                        //col = ins.nearestP.color;
                    }
                    screen.Plot(x, y, RGB2Int(col));
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

            // can displace the camera to sit at the bottom of the screen, though this does not seem necessary
            //camera.position.Z -= 5;
            //for (int i = 0; i < 4; i++)
            //{
            //    camera.screenPlane[i].Z -= 5;
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
        Vector3 EntryWiseMultiply(Vector3 vec1, Vector3 vec2)
        {
            return (vec1.X * vec2.X, vec1.Y * vec2.Y, vec1.Z * vec2.Z);
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