import java.util.concurrent.LinkedBlockingQueue;
import java.util.*;

/*
This file contains the physics calculation as described in my paper.
Author: Sebastian Weiss
*/

/*
A collision event.
Author: Sebastian Weiss
*/
class CollisionEvent
{
  float time;
  int[] spheres;
  pt[] positions;
  vec[] velocities;
  
  public CollisionEvent(float time) {
    this.time = time;
    this.spheres = new int[0];
    this.positions = new pt[0];
    this.velocities = new vec[0];
  }
  public CollisionEvent(float time, int i, pt P, vec V) {
    this.time = time;
    this.spheres = new int[]{i};
    this.positions = new pt[]{P(P)};
    this.velocities = new vec[]{V(V)};
  }
  public CollisionEvent(float time, int i1, pt P1, vec V1, int i2, pt P2, vec V2) {
    this.time = time;
    this.spheres = new int[]{i1, i2};
    this.positions = new pt[]{P(P1), P(P2)};
    this.velocities = new vec[]{V(V1), V(V2)};
  }
  public String toString() {
    String s = "t="+time;
    for (int i=0; i<spheres.length; ++i) {
      s += " (index="+spheres[i]+" pos="+positions[i]+" vel="+velocities[i]+")";
    }
    return s;
  }
}

/*
This class manages the physics thread and updates the ball positions and velocities
Author: Sebastian Weiss
*/
class BallUpdater
{
  private float halfCubeSize;
  private LinkedBlockingQueue<CollisionEvent> events;
  private CollisionEvent current;
  private BALLS balls;
  private float[] ballTimes;
  private PhysicsThread thread;
  private float tglobal;
  
  public BallUpdater(BALLS balls, float halfCubeSize) {
    this.halfCubeSize = halfCubeSize;
    reset(balls);
  }
  
  public void reset(BALLS balls) {
    if (this.thread != null) {
      this.thread.interrupt();
    }
    this.balls = new BALLS(balls);
    this.ballTimes = new float[balls.nv];
    Arrays.fill(this.ballTimes, 0);
    this.events = new LinkedBlockingQueue<CollisionEvent>(1);//balls.nv);
    this.current = null;
    this.tglobal = 0;
    this.thread = new PhysicsThread(balls, events, halfCubeSize);
    this.thread.setDaemon(true);
    this.thread.start();
  }
  
  /**
   * Calculates the physics
   * @return: the count of detected collisions between the last call to updateBalls(balls) and the next call. 
  **/
  public int calculatePhysics(float tpf) {
    int collisions = 0;
    tglobal += tpf;
    System.out.println("global time: "+tglobal);
    do {
      if (current == null) {
        System.out.println("polling next event");
        try {
          current = events.take();
        } catch (InterruptedException e) {
          System.out.println("Interrupted: "+e.getMessage());
          return 0;
        }
        System.out.println("next event: "+current);
      }
      if (current.time < tglobal) {
        collisions++;
        System.out.println("activate event at time "+current.time);
        for (int i=0; i<current.spheres.length; ++i) {
          balls.G[current.spheres[i]].set(current.positions[i]);
          balls.V[current.spheres[i]].set(current.velocities[i]);
          balls.c[current.spheres[i]] = green;
          balls.m[current.spheres[i]] = del;
          ballTimes[current.spheres[i]] = current.time;
        }
        
        current = null;
      }
    } while (current==null);
    
    return collisions;
  }
  
  /**
   * Sends the new positions and velocities to the BALLS-instance used by the rendering.
  **/
  public void updateBalls(BALLS balls) {
    assert(balls.nv == this.balls.nv);
    for (int i=0; i<balls.nv; ++i) {
      if (this.balls.m[i] > 0) {
        this.balls.m[i]--;
        if (this.balls.m[i] == 0) {
          this.balls.c[i] = cyan; 
        }
      }
      balls.G[i].set( P(this.balls.G[i],V(tglobal - ballTimes[i], this.balls.V[i])) );
      balls.V[i].set( this.balls.V[i] );
      balls.c[i] = this.balls.c[i];
      System.out.println("Sphere "+i+" pos="+balls.G[i]+" vel="+balls.V[i]);
    }
  }
  
}

/**
The physics thread.
This thread runs in parallel to the render thread. It calculates the next CollisionEvents and stores them in the provided blocking queue.
Author: Sebastian Weiss
*/
class PhysicsThread extends Thread
{
  private LinkedBlockingQueue<CollisionEvent> events;
  private BALLS balls;
  private float tglobal;
  private float s;
  
  private class Vec2i {
    private final int i,j;
    private Vec2i(int i, int j) {this.i=i; this.j=j;}
    public boolean equals(Object obj) {
      if (!(obj instanceof Vec2i)) return false;
      return (((Vec2i) obj).i == i) && (((Vec2i) obj).j == j);
    }
  }
  
  public PhysicsThread(BALLS balls, LinkedBlockingQueue<CollisionEvent> events, float s) {
    this.events = events;
    this.balls = new BALLS(balls); 
    this.s = s;
    this.tglobal = 0;
  }
  
  /*
  The main loop of the thread.
  TODO: Speed improvement from O(n^2) to O(n)
  */
  public void run() {
    int n = balls.nv;
    System.out.println("count of balls: "+n);
    float r = balls.r[0];
    int[] face = new int[1];
    ArrayList<Vec2i> zeroTimeEvents = new ArrayList<Vec2i>(); //to prevent infinit loops
    float[] timeCache = new float[n];
    int[] partnerCache = new int[n];
    Arrays.fill(timeCache, Float.POSITIVE_INFINITY);
    Arrays.fill(partnerCache, -1);
    ArrayList<Integer> invalidSpheres = new ArrayList<Integer>(); //The spheres that need to be recalculated
    ArrayList<Integer> invalidSpheres2 = new ArrayList<Integer>();
    boolean[] invalidSpheresArray = new boolean[n];
    Arrays.fill(invalidSpheresArray, false);
    
    
    System.out.println("balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
    
    
    // RUN FIRST COMPLETE COLLISION TEST
    // Sphere-Sphere collision
    for (int ii=0; ii<n-1; ++ii) {
      for (int jj=ii+1; jj<n; ++jj) {
        float t = sphereSphereCollisionTime(balls.G[ii], balls.V[ii], balls.G[jj], balls.V[jj], r);
        if (t >= 0 && t < timeCache[ii] && t < timeCache[jj]) {
          if (partnerCache[ii] >= 0 && !invalidSpheresArray[partnerCache[ii]]) {
            invalidSpheres.add(partnerCache[ii]);
            invalidSpheresArray[partnerCache[ii]] = true;
          }
          if (partnerCache[jj] >= 0 && !invalidSpheresArray[partnerCache[jj]]) {
            invalidSpheres.add(partnerCache[jj]);
            invalidSpheresArray[partnerCache[jj]] = true;
          }
          timeCache[jj] = t;
          partnerCache[jj] = ii;
          timeCache[ii] = t;
          partnerCache[ii] = jj;
        }
      }
    }
    // Sphere-Cube collision
    for (int ii=0; ii<n; ++ii) {
      float t = sphereCubeCollisionTime(balls.G[ii], balls.V[ii], r, s, face);
      if (t >= 0 && t < timeCache[ii]) {
        if (partnerCache[ii] >= 0 && !invalidSpheresArray[partnerCache[ii]]) {
          invalidSpheres.add(partnerCache[ii]);
          invalidSpheresArray[partnerCache[ii]] = true;
        }
        timeCache[ii] = t;
        partnerCache[ii] = -face[0];
      }
    }
    
    //Debug:
    System.out.println("time cache: "+Arrays.toString(timeCache));
    System.out.println("partner cache: "+Arrays.toString(partnerCache));
    System.out.println("invalid spheres: "+invalidSpheres);
    
    System.out.println("balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
    
    // LOOP: only calculate changed times
    while(!Thread.interrupted()) {
      
      System.out.println("a- balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);

      // Sphere-Sphere collision
      for (int ii : invalidSpheres) {
        for (int jj=0; jj<n; ++jj) {
          if (jj==ii) continue;
          float t = sphereSphereCollisionTime(balls.G[ii], balls.V[ii], balls.G[jj], balls.V[jj], r);
          System.out.println("check sphere "+ii+" against "+jj+", t="+t+" timeCache="+timeCache[jj]);
          if (t >= 0 && t < timeCache[jj] && !zeroTimeEvents.contains(new Vec2i(min(ii, jj), max(ii, jj)))) {
            if (partnerCache[ii] >= 0 && !invalidSpheresArray[partnerCache[ii]]) {
              timeCache[partnerCache[ii]] = Float.POSITIVE_INFINITY;
              invalidSpheres2.add(partnerCache[ii]);
              invalidSpheresArray[partnerCache[ii]] = true;
            }
            if (partnerCache[jj] >= 0 && !invalidSpheresArray[partnerCache[jj]]) {
              timeCache[partnerCache[jj]] = Float.POSITIVE_INFINITY;
              invalidSpheres2.add(partnerCache[jj]);
              invalidSpheresArray[partnerCache[jj]] = true;
            }
            timeCache[ii] = t;
            partnerCache[ii] = jj;
            timeCache[jj] = t;
            partnerCache[jj] = ii;
          }
        }
      }
      
      System.out.println("b- balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
      
      // Sphere-Cube collision
      for (int ii : invalidSpheres) {
        float t = sphereCubeCollisionTime(balls.G[ii], balls.V[ii], r, s, face);
        if (t >= 0 && t < timeCache[ii] && !zeroTimeEvents.contains(new Vec2i(-face[0], ii))) {
          if (partnerCache[ii] >= 0 && !invalidSpheresArray[partnerCache[ii]]) {
            timeCache[partnerCache[ii]] = Float.POSITIVE_INFINITY;
            invalidSpheres2.add(partnerCache[ii]);
            invalidSpheresArray[partnerCache[ii]] = true;
          }
          timeCache[ii] = t;
          partnerCache[ii] = -face[0];
        }
      }
      
      System.out.println("c- balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
      //swap invalid-spheres list
      ArrayList<Integer> tmp = invalidSpheres;
      invalidSpheres = invalidSpheres2;
      invalidSpheres2 = tmp;
      invalidSpheres2.clear();
      Arrays.fill(invalidSpheresArray, false);
      //Search next event
      float deltaT = Float.MAX_VALUE;
      int i=0, j=0;
      for (int ii=0; ii<n; ++ii) {
        if (timeCache[ii] < deltaT) {
          i = ii;
          j = partnerCache[ii];
          deltaT = timeCache[ii];
        }
      }
      //add to list to prevent infinit loops
      if (deltaT > 0)
        zeroTimeEvents.clear();
      else
        System.out.println("Zero time event");
      zeroTimeEvents.add(new Vec2i(min(i, j), max(i, j)));
      //Move all spheres to deltaT
      System.out.println("the next collision happens after "+deltaT+" timesteps between "+i+" and "+j);
      for (int ii=0; ii<n; ++ii) {
        balls.G[ii].add(deltaT, balls.V[ii]);
        timeCache[ii] -= deltaT;
      }
      tglobal += deltaT;
      
      System.out.println("d- balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
      
      //Change velocities and create event
      CollisionEvent e;
      if (j < 0) {
        System.out.print("V1="+balls.V[i]);
        balls.V[i].set(sphereCubeVelocityChange(balls.V[i], -j));
        System.out.println(" W1="+balls.V[i]+" time="+tglobal);
        e = new CollisionEvent(tglobal, i, balls.G[i], balls.V[i]);
        timeCache[i] = Float.POSITIVE_INFINITY;
        partnerCache[i] = -10;
        invalidSpheres.add(i);
        invalidSpheresArray[i] = true;
      } else {
        System.out.println("e- balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
        timeCache[i] = Float.POSITIVE_INFINITY;
        timeCache[j] = Float.POSITIVE_INFINITY;
        partnerCache[i] = -10;
        partnerCache[j] = -10;
        invalidSpheres.add(i);
        invalidSpheres.add(j);
        invalidSpheresArray[i] = true;
        invalidSpheresArray[j] = true;
        vec W1 = V();
        vec W2 = V();
        sphereSphereVelocityChange(balls.G[i], balls.V[i], balls.G[j], balls.V[j], W1, W2);
        System.out.println("distance: " + (V(balls.G[i], balls.G[j])).norm() + ", radius: " + r + ", time: " + tglobal);
        balls.V[i].set(W1);
        balls.V[j].set(W2);
        e = new CollisionEvent(tglobal, i, balls.G[i], balls.V[i], j, balls.G[j], balls.V[j]);
      }
      System.out.println("f- balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
      //System.out.println("time cache: "+Arrays.toString(timeCache));
      //System.out.println("partner cache: "+Arrays.toString(partnerCache));
      //System.out.println("invalid spheres: "+invalidSpheres);
      
      //Add event
      try {
        events.put(e);
      } catch (InterruptedException ex) {
        return;
      }
      
      System.out.println("balls.G[0]: " + balls.G[0] + ",  balls.V[0]: " + balls.V[0] + ", balls.G[1]: " + balls.G[1] + ",  balls.V[1]: " + balls.V[1]);
    }
  }
}


/*
Computes the time of the next collision between two spheres.
Author: Sebastian Weiss
*/
float sphereSphereCollisionTime(pt A1, vec V1, pt A2, vec V2, float r) {
  float Ax = A2.x-A1.x;
  float Ay = A2.y-A1.y;
  float Az = A2.z-A1.z;
  float Vx = V2.x-V1.x;
  float Vy = V2.y-V1.y;
  float Vz = V2.z-V1.z;
  float a = sq(Vx) + sq(Vy) + sq(Vz); // creating new instances is just too slow
  float b = 2 * ( Ax*Vx + Ay*Vy + Az*Vz );
  float c = sq(Ax) + sq(Ay) + sq(Az) - 4*sq(r);
  float d = sq(b) - 4*a*c;
  if (d <= 0) {
    return -1; //no solution or only one solution
  } else {
    float ds = sqrt(d);
    float t1 = (-b+ds)/(2*a);
    float t2 = (-b-ds)/(2*a);
    float ta = min(t1, t2);
    float tb = max(t1, t2);
    //if (ta < 0 && tb > 0) {
    //  System.out.println("Intersection: A1="+A1+", V1="+V1+", A2="+A2+", V2="+V2+", r="+r+" -> d="+V(A1, A2).norm()+", t1="+ta+", t2="+tb);
    //  return Float.NaN;
    //}
    return ta;
  }
}

vec project(vec X, vec N) {
  //float s = (X.x*N.x + X.y*N.y + X.z*N.z) / (sq(N.x) + sq(N.y) * sq(N.z));
  //return V(s, N);
  vec NN = V(N).normalize();
  return NN.mul(X.x*NN.x + X.y*NN.y + X.z*NN.z);
}

/*
Computes the new velocities after a collision between two spheres.
I think the formulas are not correct, they produce some strange behaviour.
Author: Sebastian Weiss
*/
void sphereSphereVelocityChange(pt A1, vec V1, pt A2, vec V2, vec W1, vec W2) {
  vec N = V(A1, A2);
  W1.set( V1.sub(project(V1, N)).add(project(V2, N)) );
  W2.set( V2.sub(project(V2, N)).add(project(V1, N)) );
  System.out.println("A1="+A1+" A2="+A2+" V1="+V1+" V2="+V2+" W1="+W1+" W2="+W2);
  float s1 = V1.norm() + V2.norm();
  float s2 = W1.norm() + W2.norm();
  if (abs(s1-s2) > 0.0001f) {
    System.err.println("Velocity change out of bounds");
  }
}

/*
Computes the time of the next collision between a sphere and the surrounding cube.
Author: Sebastian Weiss
*/
float sphereCubeCollisionTime(pt A, vec V, float r, float s, int[] face) {
  float tx1 = (-s + r - A.x) / V.x;
  float tx2 = (s - r - A.x) / V.x;
  float ty1 = (-s + r - A.y) / V.y;
  float ty2 = (s - r - A.y) / V.y;
  float tz1 = (-s + r - A.z) / V.z;
  float tz2 = (s - r - A.z) / V.z;
  float t = Float.MAX_VALUE;
  if (V.x<0 && tx1 < t && tx1 >= 0) {
    t = tx1; face[0] = 1;
  }
  if (V.x > 0 && tx2 < t && tx2 >= 0) {
    t = tx2; face[0] = 2;
  }
  if (V.y<0 && ty1 < t && ty1 >= 0) {
    t = ty1; face[0] = 3;
  }
  if (V.y>0 && ty2 < t && ty2 >= 0) {
    t = ty2; face[0] = 4;
  }
  if (V.z<0 && tz1 < t && tz1 >= 0) {
    t = tz1; face[0] = 5;
  }
  if (V.z>0 && tz2 < t && tz2 >= 0) {
    t = tz2; face[0] = 6;
  }
  return t;
}

/*
Changes the velocity of the sphere after a collision with the cube.
Author: Sebastian Weiss
*/
vec sphereCubeVelocityChange(vec V, int face) {
  switch (face) {
    case 1:
    case 2:
      return V(-V.x, V.y, V.z);
    case 3:
    case 4:
      return V(V.x, -V.y, V.z);
    case 5:
    case 6:
      return V(V.x, V.y, -V.z);
    default:
      throw new IllegalArgumentException("Unknown face "+face);
  }
}