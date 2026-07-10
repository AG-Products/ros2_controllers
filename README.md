<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->


<!-- PROJECT LOGO -->
<br />
<div align="center">

<h3 align="center">AG-Products/ros_controllers</h3>

  <p align="center">
    This is a stripped-down fork of the <a href="https://github.com/ros-controls/ros2_controllers">ros2_controllers</a> package, with modifications to add features and ensure the desired functionality.
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This is a stripped-down fork of the <a href="https://github.com/ros-controls/ros2_controllers">ros2_controllers</a> package, with modifications to add features and ensure the desired functionality with the AG ARC.
List of changes from the original ros2_controllers:
- steering_controllers_library:
  - Add limits to the velocities of the left and right wheels. This is implemented by reducing the desired forward velocity `v_bx` if one or both of the wheels would exceed the specified limit
  - Implemented in-place rotation as this was missing from the humble version that we installed
- tricycle_steering_controller
  - Addition of parameter for wheel velocity limits `wheel_velocity_limit`

<p align="right">(<a href="#readme-top">back to top</a>)</p>




<!-- GETTING STARTED -->
## Getting Started

### Prerequisites
Ensure that the two packages modified by AG are not installed via `apt`:
  ```
  sudo apt remove ros-humble-tricycle-steering-controller ros-humble-steering-controllers-library
  ```
### Installation
Cloning the repository requires having an authenticated ssh token with an account that has access. See [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
1. `mkdir -p \<workspace>/src`
2. `cd \<workspace>/src`
3. `git clone git@github.com:AG-Products/ros2_controllers.git --branch humble`
4. `cd ..`
5. `rosdep update`
6. `rosdep install --from-paths src --ignore-src -r -y`
7. `colcon build`
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Usage is the same as the standard tricycle_steering_controller, and the parameters available can be found in [tricycle_steering_controller/test/tricycle_steering_controller_params.yaml](tricycle_steering_controller/test/tricycle_steering_controller_params.yaml).

1. Create a `params.yaml` file with the following setup:
```
controller_manager:
  ros__parameters:
    tricycle_controller:
      type: tricycle_steering_controller/TricycleSteeringController
```
2. Add any params needed to configure the tricycle controller:
```
tricycle_controller:
  ros__parameters:
    \<your_params_here>
```
1. Bringup the controller manager using a launch file. An example launch file can be found in [the zapi hardware package](https://github.com/AG-Products/robot_pkgs/tree/chris_trike_updates).
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

- [ ] With the current implementation of in-place rotation, the robot must stop and wait for the rear wheel to reach 90 degrees before rotation begins. A smoother entry to in-place rotation might be nice one day.

See the [open issues](https://github.com/AG-Products/ros2_controllers/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/AG-Products/ros2_controllers.svg?style=for-the-badge
[contributors-url]: https://github.com/AG-Products/ros2_controllers/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/AG-Products/ros2_controllers.svg?style=for-the-badge
[forks-url]: https://github.com/AG-Products/ros2_controllers/network/members
[stars-shield]: https://img.shields.io/github/stars/AG-Products/ros2_controllers.svg?style=for-the-badge
[stars-url]: https://github.com/AG-Products/ros2_controllers/stargazers
[issues-shield]: https://img.shields.io/github/issues/AG-Products/ros2_controllers.svg?style=for-the-badge
[issues-url]: https://github.com/AG-Products/ros2_controllers/issues
[license-shield]: https://img.shields.io/github/license/AG-Products/ros2_controllers.svg?style=for-the-badge
[license-url]: https://github.com/AG-Products/ros2_controllers/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
<!-- Shields.io badges. You can a comprehensive list with many more badges at: https://github.com/inttter/md-badges -->
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 