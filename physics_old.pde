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
    this.events = new LinkedBlockingQueue<CollisionEvent>(50);
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
    //System.out.println("global time: "+tglobal);
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
      //System.out.println("Sphere "+i+" pos="+balls.G[i]+" vel="+balls.V[i]);
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
    while(!Thread.interrupted()) {
      int i=0;
      int j=0;
      float deltaT = Float.MAX_VALUE;
      // Sphere-Sphere collision
      for (int ii=0; ii<n-1; ++ii) {
        for (int jj=ii+1; jj<n; ++jj) {
          float t = sphereSphereCollisionTime(balls.G[ii], balls.V[ii], balls.G[jj], balls.V[jj], r);
          //System.out.println("sphere "+ii+" and "+jj+" collide at time "+t);
          if (t >= 0 && t < deltaT && !zeroTimeEvents.contains(new Vec2i(ii, jj))) {
            deltaT = t;
            i = ii;
            j = jj;
          }
        }
      }
      // Sphere-Cube collision
      for (int ii=0; ii<n; ++ii) {
        float t = sphereCubeCollisionTime(balls.G[ii], balls.V[ii], r, s, face);
        //System.out.println("sphere "+i+" collides with wall "+face[0]+" at time "+t);
        if (t >= 0 && t < deltaT && !zeroTimeEvents.contains(new Vec2i(ii, -face[0]))) {
          deltaT = t;
          i = ii;
          j = -face[0];
        }
      }
      //add to list to prevent infinit loops
      if (deltaT > 0)
        zeroTimeEvents.clear();
      zeroTimeEvents.add(new Vec2i(i, j));
      //Move all spheres to deltaT
      System.out.println("the next collision happens after "+deltaT+" timesteps between "+i+" and "+j);
      for (int ii=0; ii<n; ++ii) {
        balls.G[ii].add(deltaT, balls.V[ii]);
      }
      tglobal += deltaT;
      
      //Test
      //if (j < 0) {
      //  pt P = balls.G[i];
      //  float d = min(s-P.x, min(s-P.y, min(s-P.z, min(s+P.x, min(s+P.y, s+P.z)))));
      //  System.out.println("expected distance: "+r+", actual distance: "+d);
      //} else {
      //  float d = V(balls.G[i], balls.G[j]).norm();
      //  System.out.println("expected distance: "+(2*r)+", actual distance: "+d);
      //}
      
      //Change velocities and create event
      CollisionEvent e;
      if (j < 0) {
        balls.V[i].set(sphereCubeVelocityChange(balls.V[i], -j));
        e = new CollisionEvent(tglobal, i, balls.G[i], balls.V[i]);
      } else {
        vec W1 = V();
        vec W2 = V();
        sphereSphereVelocityChange(balls.G[i], balls.V[i], balls.G[j], balls.V[j], W1, W2);
        balls.V[i].set(W1);
        balls.V[j].set(W2);
        e = new CollisionEvent(tglobal, i, balls.G[i], balls.V[i], j, balls.G[j], balls.V[j]);
      }
      
      //Add event
      try {
        events.put(e);
      } catch (InterruptedException ex) {
        return;
      }
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