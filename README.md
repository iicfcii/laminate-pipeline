# Laminate Pipeline
This project aims to enable designing laminate robot using [Fusion 360](https://www.autodesk.com/products/fusion-360/overview).

## Design in Fusion 360
### Examples
It is best to try the entire process with one of these known working designs first.
* [1-layer Fourbar](https://a360.co/3m8pc8i)
* [5-layer Fourbar](https://a360.co/326is3y)

### Steps
Here are the general steps for designing your own laminate robots.
* Define materials thickness in the *Parameters* (*Solid*->*Modify*->*Change Parameters*). This step makes it easier when extruding layers later.
* Draw the design as multiple bodies on the **XY plane**.  
* Convert bodies into components using *New Component*. You might want to set one of the component as *Ground*.
* Create *Revolute Joints* or *Revolute As-built Joints*. This represents where your design can be folded or rotated. Please make sure vertically the joint is located at the center of the intended flexible layer.  
* Validate your design by folding the components into desired shape. If you want to add other non-laminate parts such as motors and PCBs, please create another design and import this to there. This way, other parts won't interfere with latter scripts.  

## Python Environment
The scripts are written in Python. Please install [Miniconda](https://docs.conda.io/en/latest/miniconda.html)/[Anaconda](https://www.anaconda.com/) to manage the python environment and required software packages. After installation, open an "Anaconda Prompt" and use the following commands.

Create a Python 3.8 environment.
```
conda create --name laminate python=3.8
```

Install [foldable_robotics](https://github.com/idealabasu/code_foldable_robotics) package.
```
pip install foldable_robotics
```

You may need to install shapely manually.
```
conda install shapely
```

Please either clone or download and extract this repository to your desired folder.

## Export from Fusion 360
* In Fusion 360, make sure your design is unfolded. You can drag the slider at the bottom to quickly go to a state where all components and joints are created and design is still flat.  
* Under the *Scripts and Add-Ins* (*Tools*->*Add-Ins*->*Scripts and Add-Ins*), click the green plus sign to the right of *My Scripts* and select the "SaveLaminate" folder in this repository.
* Double click the "SaveLaminate" and follow the instructions on screen.
* The script will automatically convert the design and save some dxf and csv files into the folder you just selected. A complicated design might take a while to process.

## Generate cut files
* In the "Anaconda Prompt", use the following command.
```
python PATH\TO\REPOSITORY\main.py PATH\TO\EXPORT\FOLDER
```
* After a while, two additional files named "EXPORT_FOLDER_NAME_layers.dxf" and "EXPORT_FOLDER_NAME_release.dxf" should be added to the export folder. You can open them with Adobe Illustrator, Inkscape, or other software

## Fabricate
TBD
