using System;
using System.Collections.Generic;

namespace FlatPhysics
{
    public sealed class FlatWorld
    {
        public static int TransformCount = 0;
        public static int NoTransformCount = 0;

        public static readonly float MinBodySize = 0.01f * 0.01f;
        public static readonly float MaxBodySize = 64f * 64f;

        public static readonly float MinDensity = 0.5f;     // g/cm^3
        public static readonly float MaxDensity = 21.4f;

        public static readonly int MinIterations = 1;
        public static readonly int MaxIterations = 128;

        private FlatVector gravity;
        private List<FlatBody> bodyList;
        private List<FlatManifold> contactList;

        public List<FlatVector> ContactPointsList;

        public int BodyCount
        {
            get { return this.bodyList.Count; }
        }

        public FlatWorld()
        {
            this.gravity = new FlatVector(0f, -9.81f);
            this.bodyList = new List<FlatBody>();
            this.contactList = new List<FlatManifold>();

            this.ContactPointsList = new List<FlatVector>();
        }

        public void AddBody(FlatBody body)
        {
            this.bodyList.Add(body);
        }

        public bool RemoveBody(FlatBody body)
        {
            return this.bodyList.Remove(body);
        }

        public bool GetBody(int index, out FlatBody body)
        {
            body = null;

            if(index < 0 || index >= this.bodyList.Count)
            {
                return false;
            }

            body = this.bodyList[index];
            return true;
        }

        public void Step(float time, int iterations)
        {
            iterations = FlatMath.Clamp(iterations, FlatWorld.MinIterations, FlatWorld.MaxIterations);

            this.ContactPointsList.Clear();

            for (int it = 0; it < iterations; it++)
            {
                // Movement step
                for (int i = 0; i < this.bodyList.Count; i++)
                {
                    this.bodyList[i].Step(time, this.gravity, iterations);
                }

                this.contactList.Clear();

                // collision step
                for (int i = 0; i < this.bodyList.Count - 1; i++)
                {
                    FlatBody bodyA = this.bodyList[i];
                    FlatAABB bodyA_aabb = bodyA.GetAABB();

                    for (int j = i + 1; j < this.bodyList.Count; j++)
                    {
                        FlatBody bodyB = this.bodyList[j];
                        FlatAABB bodyB_aabb = bodyB.GetAABB();

                        if (bodyA.IsStatic && bodyB.IsStatic)
                        {
                            continue;
                        }

                        if (!Collisions.IntersectAABBs(bodyA_aabb, bodyB_aabb))
                        {
                            continue;
                        }

                        if (Collisions.Collide(bodyA, bodyB, out FlatVector normal, out float depth))
                        {
                            if (bodyA.IsStatic)
                            {
                                bodyB.Move(normal * depth);
                            }
                            else if (bodyB.IsStatic)
                            {
                                bodyA.Move(-normal * depth);
                            }
                            else
                            {
                                bodyA.Move(-normal * depth / 2f);
                                bodyB.Move(normal * depth / 2f);
                            }

                            Collisions.FindContactPoints(bodyA, bodyB, out FlatVector contact1, out FlatVector contact2, out int contactCount);
                            FlatManifold contact = new FlatManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                            this.contactList.Add(contact);
                        }
                    }
                }

                for(int i = 0; i < this.contactList.Count; i++)
                {
                    FlatManifold contact = this.contactList[i];
                    this.ResolveCollision(in contact);

                    if (contact.ContactCount > 0)
                    {
                        if (!this.ContactPointsList.Contains(contact.Contact1))
                        {
                            this.ContactPointsList.Add(contact.Contact1);
                        }

                        if (contact.ContactCount > 1)
                        {
                            if (!this.ContactPointsList.Contains(contact.Contact2))
                            {
                                this.ContactPointsList.Add(contact.Contact2);
                            }
                        }
                    }
                }
            }
        }

        public void ResolveCollision(in FlatManifold contact)
        {
            FlatBody bodyA = contact.BodyA;
            FlatBody bodyB = contact.BodyB;
            FlatVector normal = contact.Normal;
            float depth = contact.Depth;

            FlatVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

            if (FlatMath.Dot(relativeVelocity, normal) > 0f)
            {
                return;
            }

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float j = -(1f + e) * FlatMath.Dot(relativeVelocity, normal);
            j /= bodyA.InvMass + bodyB.InvMass;

            FlatVector impulse = j * normal;

            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyB.LinearVelocity += impulse * bodyB.InvMass;
        }


    }
}
