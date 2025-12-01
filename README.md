<img src="Toolbox/TradeoffToolbox.png" height="150"/>

## Name
Garpinger's Trade-Off Diagram Toolbox for PID Control Design

## Description
A MATLAB project implementing Garpinger’s trade-off diagram approach for PID controller tuning and performance analysis. The project provides core functions for trade-off visualization, a live script showcasing a real-world field study, and a detailed explanation of the underlying theory.

## Installation
The Toolbox is available for Matlab R2025a or later.
It is provided via

Option 1:  
a) From Matlab -> Apps -> Get More Apps -> Search for "Trade-Off Diagram Toolbox for PID Control Design" -> Add -> Add to Matlab  
  
This will download the toolbox to your local installation and set the Matlab path appropriately.  
  
Option 2:  
a) From Matlab -> Apps -> Get More Apps -> Search for "Trade-Off Diagram Toolbox for PID Control Design" -> Add -> Download Only  
b) extract the contents of the downloaded zip-archive to a local folder  
c) Either open the Matlab-project file "TradeoffToolbox.prj" or set the Matlab Path to include  
   - JRC-ISIA-matlab-addon-pid-garpingers-tradeoff-diagrams-VERSION  
   - JRC-ISIA-matlab-addon-pid-garpingers-tradeoff-diagrams-VERSION/Toolbox  
   - JRC-ISIA-matlab-addon-pid-garpingers-tradeoff-diagrams-VERSION/Toolbox/Helpers  
   - JRC-ISIA-matlab-addon-pid-garpingers-tradeoff-diagrams-VERSION/Toolbox/doc  
   - JRC-ISIA-matlab-addon-pid-garpingers-tradeoff-diagrams-VERSION/Toolbox/Examples  
   - JRC-ISIA-matlab-addon-pid-garpingers-tradeoff-diagrams-VERSION/Toolbox/Data  
  
Option 3:  
a) go to https://www.mathworks.com/matlabcentral/fileexchange/ -> search for "Trade-Off Diagram Toolbox for PID Control Design" and download the provided zip-archive  
b)+c): see Option 2  

## Usage
The recommended entry point for using this toolbox is the Live Script ``GettingStarted``. After successfully installing the Toolbox, it will open automatically and provide a step-by-step introduction to the Toolbox.

Another option is to open the Live script ``FieldstudyTradeoff``. This script demonstrates a complete field study conducted on a DC motor. It includes a structured description of the study setup, followed by the application of core functions provided by the toolbox.

The script serves both as a practical example and as a template for adapting the methodology to other systems. It is particularly suitable for users who prefer a guided, interactive approach to exploring the toolbox's capabilities.

All individual functions within the toolbox are documented and can be explored using MATLAB’s built-in help system. To access the documentation for a specific function, use the commands  
``help calculateTradeoff``  
``help calculateCascadedTradeoff``  
``help plotTradeoff``  

## Cite As
Hoher, S. & Rehrl, J., (2025) "Garpinger's Trade-Off Diagram Toolbox for PID Control Design", MATLAB Central File Exchange.  
Hoher, S. & Rehrl, J., (2025) “Analysis and tuning of PID controller gains for DC servo drives using Garpinger’s trade-off plots”, ARW Proceedings 25(1), 19-24. doi: https://doi.org/10.34749/3061-0710.2025.3
