# Building a Simple Physics Scene

This note covers how to place objects in a scene, add physics, configure contact/friction materials, and assign visual materials in NVIDIA Isaac Sim.

In this example:

- **Cube** → robot **Body**
- **Cylinders** → left / right **Wheels**

## 1. Add Objects to the Scene

1. Create three Xforms and rename them to Body, Wheel_left, and Wheel_right.

2. Create a Cube for the body and two Cylinders for the wheels.

3.Adjust each mesh object's translate, rotate, and scale.

4.Drag the Cube and Cylinders under the corresponding Xform in the Stage tree 

# 2. Add Physics Properties
1. In the Stage tree, multi-select the Body (Cube) and both Wheels (Cylinders):
   - Hold **Ctrl** and click each prim, or
   - Hold **Shift** if they are listed consecutively.
2. Open the **Property** panel and click **+ Add**.
3. Choose **Physics > Rigid Body with Colliders Preset**.
4. Press **Play** and confirm that all three objects fall to the ground.

### What this preset does

**Rigid Body with Colliders Preset** applies both:

- **Rigid Body API** — the object has mass and responds to gravity/forces  
- **Collision API** — the object participates in collision detection  

## 3. Add Contact and Friction Parameters

1. From the menu bar, go to **Create > Physics > Physics Material**.
2. Choose **Rigid Body Material**. A new `PhysicsMaterial` prim appears in the Stage tree.
3. Tune the parameters such as friction coefficients and restitution in its property tab.
   
### Apply a physics material to an object

1. Select the target object (Body or a Wheel) in the Stage tree.
2. In the Property panel, find **Materials on Selected Model**.
3. Choose the physics material from the dropdown.

## 4. Assign Visual Materials

1. Go to **Create > Materials > OmniPBR**.
2. Right-click the new material in the Stage tree and rename it (for example `Body_Mat`, `Wheel_Mat`).
3. Select Body or a Wheel, then in **Materials on Selected Model**, pick the matching material.
4. Edit the material properties (for example **Shader / Albedo**, roughness, reflectivity) until the appearance looks right.
 
