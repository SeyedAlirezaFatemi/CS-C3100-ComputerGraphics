# CS-C3100 Computer Graphics, Fall 2021
# Lehtinen / Aho, Kaskela, Kynkäänniemi
#
# Assignment 1: Introduction

Student name: Seyed Alireza FatemiJahromi

R1 Moving an object                          (1p): done
R2 Generating a simple cone mesh and normals (3p): done
R3 Converting mesh data for OpenGL viewing   (3p): done
R4 Loading a large mesh from file            (3p): done

+

Version control
Rotate and scale transforms
Transforming normals in vertex shader + Efficiently transforming normals in vertex shader
Better camera
Animation
Viewport correction + Viewport and perspective
Add support for loading another file format (ASCII PLY)
Mesh Simplification (in App::simplifyMesh):
    "Surface Simplification Using Quadric Error Metrics". Press 5 and load a ply file.
    The code simplifies the mesh by removing 100 faces. You can change this in the code.
    This is not a perfect implementation since it does not perform checks before and after edge collapses.
