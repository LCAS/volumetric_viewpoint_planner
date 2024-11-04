## Volumetric Viewpoint Planner

This repository generates a set of viewpoints in a volume to scan an object. The volume surface and the point scattering features can be specified by user. The packages are designed especially for a robot arm scanning a plant with a camera to complete perception tasks, such as 3D resconstruction, segmentation, and selective harvesting. The repository can generate viewpoints for the arm, command them randomly or within an order and save RGB images when the arm reaches to a viewpoint. 

The workflow of the repository is illustrated below.

<img src="docs/Volumetric_Viewpoint_Planner_Github.png" width="800" > 

### Installation/Running

1. **Open in Visual Studio Code:**

   Open the cloned repository in VSCode. VSCode will prompt you to "Reopen in Container." Alternatively, you can use the command palette (`Ctrl+Shift+P`) and search for the "reopen in container" command.

   <img src="https://github.com/LCAS/ros2_pkg_template/assets/47870260/52b26ae9-ffe9-4e7c-afb9-88cee88f870f" width="300">


   Then this will promote you with the following two options:

   <img src="https://github.com/user-attachments/assets/d0885c75-59de-4b5d-a8b7-c38bf02444d4" width="400">


   You may select the base image according to your targeted application. For instance, if the nodes do not require GPU processing tasks, it is preferable to use the default devcontainer as it is more lightweight.

2. **Accessing the Desktop Interface:**
   Open the user interface by navigating to the PORTS tab in VSCode, selecting port `6080` (or port `5801` for the CUDA-OpenGL version), and opening it in the browser.

   <img src="https://github.com/LCAS/ros2_pkg_template/assets/47870260/b61f4c95-453b-4c92-ad66-5133c91abb05" width="400">



