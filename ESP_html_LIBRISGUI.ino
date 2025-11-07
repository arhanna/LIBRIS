/*
 * ESP32 GUI Code - Synchronized with 20251031_generaloutputcollect_libgen_original.ino
 * 
 * Key Synchronizations:
 * - Regulator count: Updated to 9 regulators (6 active: AS/BS/CS/DS/ES/FS, 3 reserved: GS/HS/IS)
 * - Data structure: Sends 4 formulations x 9 regulators (last 3 always 0)
 * - Format: Matches 20251031 structure (AS, BS, CS, DS, ES, FS, GS, HS, IS)
 * 
 * Note: This code sends only the 4 formulations (no priming rows).
 * The 20251031 code has 2 priming rows before formulations (indices 0-1),
 * then 4 formulations (indices 2-5). This ESP code sends only indices 2-5.
 */

#include <WiFi.h>
#include <HardwareSerial.h>

HardwareSerial mySerial(0); // Use UART0 (TX = GPIO 21, RX = GPIO 20)

const char* ssid = "a pint of plain";
const char* password = "";  // No password
// Updated to match 20251031: 9 regulators per formulation
// Regulators: AS(0), BS(1), CS(2), DS(3), ES(4), FS(5), GS(6), HS(7), IS(8)
// Mapping: E1->AS, E2->BS, E3->CS, A1->DS, A2->ES, A3->FS, GS/HS/IS reserved (0)
float E1F1 = 0;  // AS
float E2F1 = 0;  // BS
float E3F1 = 0;  // CS
float A1F1 = 0;  // DS
float A2F1 = 0;  // ES
float A3F1 = 0;  // FS
float G1F1 = 0;  // GS (reserved, set to 0)
float H1F1 = 0;  // HS (reserved, set to 0)
float I1F1 = 0;  // IS (reserved, set to 0)

float E1F2 = 0;
float E2F2 = 0;
float E3F2 = 0;
float A1F2 = 0;
float A2F2 = 0;
float A3F2 = 0;
float G1F2 = 0;
float H1F2 = 0;
float I1F2 = 0;

float E1F3 = 0;
float E2F3 = 0;
float E3F3 = 0;
float A1F3 = 0;
float A2F3 = 0;
float A3F3 = 0;
float G1F3 = 0;
float H1F3 = 0;
float I1F3 = 0;

float E1F4 = 0;
float E2F4 = 0;
float E3F4 = 0;
float A1F4 = 0;
float A2F4 = 0;
float A3F4 = 0;
float G1F4 = 0;
float H1F4 = 0;
float I1F4 = 0;


const int rows = 4;
const int cols = 9;  // Updated to match 20251031: 9 regulators (6 active + 3 reserved)
// Add other variables as needed

const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>LNP Library Generator</title>
    <style>
        .container {
            margin: 20px;
            text-align: center; /* Center align content */
        }
        .button-container {
          display: flex;
          align-items: center;
          margin-bottom: 10px;
		  justify-content: center;
        }

        .button {
            margin-right: 10px;
        }
        
        .home_button{
        	margin-top: 20px;
        }

        .pressure-container {
            display: flex;
            align-items: center;
            margin-left: 10px; /* Adjust margin as needed */
        }

        .pressure-label {
            margin-right: 5px;
        }

        .pressure-window {
            width: 60px; /* Adjust width as needed */
            border: 1px solid #ccc;
            padding: 5px;
            text-align: center;
            
        }
        .textbox{
        	margin-top: 10px;
        }
        .textbox-row {
            display: flex;
            justify-content: center; /* Center align items horizontally */
        }
        .textbox-row .textbox-container {
            margin: 10px;
            input{
            	text-align: center; /* Center align content */
                width: 150px;
                height: 15px;
            }
            margin-top:20px;
            margin-bottom: 50px;
        }
        .header {
            font-weight: bold;
            font-size: 24px; /* Larger font size for main header */
        }
        .formulation-header {
            font-size: 18px; /* Smaller font size for formulation headers */
            margin-top: 10px;
        }
        .matrix-container {
            margin-top: 20px;
            text-align: center; /* Center align matrix */
        }
        .matrix-table {
            width: 80%;
            border-collapse: collapse;
            margin: auto;
            width = 50px;
        }
        .matrix-table th, .matrix-table td {
            border: 1px solid #ccc;
            padding: 10px;
            width = 250px;
        }
        .hidden {
            display: none;
        }
        .additional-buttons {
          display: flex;
          justify-content: center;
          margin-top: 20px;
        }

        .additional-buttons .button {
          margin: 0 10px;
          padding: 10px 20px;
          font-size: 16px;
          border-radius: 5px;
          cursor: pointer;
          margin-top: 30px;
        }

        .send-button {
          background-color: green;
          color: white;
          border: none;
        }

        .terminate-button {
          background-color: red;
          color: white;
          border: none;
        }
    </style>
</head>
<body>
    <div id="mainPage" class="container">
        <header>
            <h1 class="header">LNP Library Generator</h1>
            <p class="formulation-header">Enter Input Concentrations (mg/ml) in the Top Box and MW in the Bottom Box (g/mol)</p>
        </header>
        <div class="textbox-row">
            <div class="textbox-container">
                <label for="textbox1" text-align:>IL</label><br>
                <input type="text" id="textbox1" class="textbox" oninput="updateValues('0')"><br>
                <input type="text" id="textbox5" class="textbox" oninput="updateValues('0')">
            </div>
            <div class="textbox-container">
                <label for="textbox2">PHOS</label><br>
                <input type="text" id="textbox2" class="textbox" oninput="updateValues('0')"><br>
                <input type="text" id="textbox6" class="textbox" oninput="updateValues('0')">
            </div>
            <div class="textbox-container">
                <label for="textbox3">CHOL</label><br>
                <input type="text" id="textbox3" class="textbox" oninput="updateValues('0')"><br>
                <input type="text" id="textbox7" class="textbox" oninput="updateValues('0')">
            </div>
            <div class="textbox-container">
                <label for="textbox4">PEG</label><br>
                <input type="text" id="textbox4" class="textbox" oninput="updateValues('0')"><br>
                <input type="text" id="textbox8" class="textbox" oninput="updateValues('0')">
            </div>
        </div>
        <div class="button-container">
            <button class="button" onclick="showFormulation('formulation1')">Formulation 1</button>
            <div class="pressure-container">
                <label class="pressure-label" for="pressure1-1-label">IL/CHOL Pressure (PSI):</label>
                <input type= "text" id="pressure1-1" class="pressure-window"><br>
            </div>
            <div class="pressure-container">
                <label class="pressure-label" for="pressure1-2-label">PHOS/PEG Pressure (PSI):</label>
                <input type= "text" id="pressure1-2" class="pressure-window"><br>
            </div>
        </div>
        <div class="button-container">
            <button class="button" onclick="showFormulation('formulation2')">Formulation 2</button>
            <div class="pressure-container">
              <label class="pressure-label" for="pressure2-1-label">IL/CHOL Pressure (PSI):</label>
              <input type= "text" id="pressure2-1" class="pressure-window"><br>
            </div>
            <div class="pressure-container">
              <label class="pressure-label" for="pressure2-2-label">PHOS/PEG Pressure (PSI):</label>
              <input type= "text" id="pressure2-2" class="pressure-window"><br>
            </div>
        </div>
        <div class="button-container">
            <button class="button" onclick="showFormulation('formulation3')">Formulation 3</button>
            <div class="pressure-container">
              <label class="pressure-label" for="pressure3-1-label">IL/CHOL Pressure (PSI):</label>
              <input type= "text" id="pressure3-1" class="pressure-window"><br>
            </div>
            <div class="pressure-container">
              <label class="pressure-label" for="pressure3-2-label">PHOS/PEG Pressure (PSI):</label>
              <input type= "text" id="pressure3-2" class="pressure-window"><br>
            </div>
        </div>
        <div class="button-container">
            <button class="button" onclick="showFormulation('formulation4')">Formulation 4</button>
            <div class="pressure-container">
              <label class="pressure-label" for="pressure4-1-label">IL/CHOL Pressure (PSI):</label>
              <input type= "text" id="pressure4-1" class="pressure-window"><br>
            </div>
            <div class="pressure-container">
              <label class="pressure-label" for="pressure4-2-label">PHOS/PEG Pressure (PSI):</label>
              <input type= "text" id="pressure4-2" class="pressure-window"><br>
            </div>
        </div>
        <div class="additional-buttons">
            <button class="button send-button" onclick="sendData('button', 1)">Send Data</button>
            <button class="button terminate-button" onclick="sendData('terminate', 1)">Terminate</button>
        </div>
    </div>

    <div id="formulation1" class="container hidden">
        <h1>Formulation 1</h1>
        <label for="slider1-1">IL/CHOL Pressure (PSI)</label>
        <input type="range" id="slider1-1" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('1')">
        <input type="number" id="slider1-1-value" class="slider-value" min="0" max="50" value="0"><br>
        <label for="slider1-2">PHOS/PEG Pressure (PSI)</label>
        <input type="range" id="slider1-2" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('1')">
        <input type="number" id="slider1-2-value" class="slider-value" min="0" max="50" value="0"><br>
        <div class="matrix-container">
            <table class="matrix-table">
                <thead>
                    <tr>
                        <th>IL</th>
                        <th>Phospholipid</th>
                        <th>Cholesterol</th>
                        <th>PEG</th>
                    </tr>
                </thead>
                <label for="matrix1-body">Composition Matrix (Molar Percent)</label>
                <tbody id="matrix1-body">
                    <tr>
                        <td id="1-row1-col1">0.00</td>
                        <td id="1-row1-col2">0.00</td>
                        <td id="1-row1-col3">0.00</td>
                        <td id="1-row1-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="1-row2-col1">0.00</td>
                        <td id="1-row2-col2">0.00</td>
                        <td id="1-row2-col3">0.00</td>
                        <td id="1-row2-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="1-row3-col1">0.00</td>
                        <td id="1-row3-col2">0.00</td>
                        <td id="1-row3-col3">0.00</td>
                        <td id="1-row3-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="1-row4-col1">0.00</td>
                        <td id="1-row4-col2">0.00</td>
                        <td id="1-row4-col3">0.00</td>
                        <td id="1-row4-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="1-row5-col1">0.00</td>
                        <td id="1-row5-col2">0.00</td>
                        <td id="1-row5-col3">0.00</td>
                        <td id="1-row5-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="1-row6-col1">0.00</td>
                        <td id="1-row6-col2">0.00</td>
                        <td id="1-row6-col3">0.00</td>
                        <td id="1-row6-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="1-row7-col1">0.00</td>
                        <td id="1-row7-col2">0.00</td>
                        <td id="1-row7-col3">0.00</td>
                        <td id="1-row7-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="1-row8-col1">0.00</td>
                        <td id="1-row8-col2">0.00</td>
                        <td id="1-row8-col3">0.00</td>
                        <td id="1-row8-col4">0.00</td>
                    </tr>
                </tbody>
            </table>
        </div>
        <button class="home_button" onclick="showMainPage()">Back to Main Page</button>
    </div>

    <div id="formulation2" class="container hidden">
        <h1>Formulation 2</h1>
        <label for="slider2-1">IL/CHOL Pressure (PSI)</label>
        <input type="range" id="slider2-1" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('2')">
        <input type="number" id="slider2-1-value" class="slider-value" min="0" max="50" value="0" onchange="updateValues('2')"><br>
        <label for="slider2-2">PHOS/PEG Pressure (PSI)</label>
        <input type="range" id="slider2-2" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('2')">
        <input type="number" id="slider2-2-value" class="slider-value" min="0" max="50" value="0" onchange="updateValues('2')"><br>
        <div class="matrix-container">
            <table class="matrix-table">
                <thead>
                    <tr>
                        <th>IL</th>
                        <th>Phospholipid</th>
                        <th>Cholesterol</th>
                        <th>PEG</th>
                    </tr>
                </thead>
                <label for="matrix2-body">Composition Matrix (Molar Percent)</label>
                <tbody id="matrix2-body">
                    <tr>
                        <td id="2-row1-col1">0.00</td>
                        <td id="2-row1-col2">0.00</td>
                        <td id="2-row1-col3">0.00</td>
                        <td id="2-row1-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="2-row2-col1">0.00</td>
                        <td id="2-row2-col2">0.00</td>
                        <td id="2-row2-col3">0.00</td>
                        <td id="2-row2-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="2-row3-col1">0.00</td>
                        <td id="2-row3-col2">0.00</td>
                        <td id="2-row3-col3">0.00</td>
                        <td id="2-row3-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="2-row4-col1">0.00</td>
                        <td id="2-row4-col2">0.00</td>
                        <td id="2-row4-col3">0.00</td>
                        <td id="2-row4-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="2-row5-col1">0.00</td>
                        <td id="2-row5-col2">0.00</td>
                        <td id="2-row5-col3">0.00</td>
                        <td id="2-row5-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="2-row6-col1">0.00</td>
                        <td id="2-row6-col2">0.00</td>
                        <td id="2-row6-col3">0.00</td>
                        <td id="2-row6-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="2-row7-col1">0.00</td>
                        <td id="2-row7-col2">0.00</td>
                        <td id="2-row7-col3">0.00</td>
                        <td id="2-row7-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="2-row8-col1">0.00</td>
                        <td id="2-row8-col2">0.00</td>
                        <td id="2-row8-col3">0.00</td>
                        <td id="2-row8-col4">0.00</td>
                    </tr>
                </tbody>
            </table>
        </div>
        <button class="home_button" onclick="showMainPage()">Back to Main Page</button>
    </div>

    <div id="formulation3" class="container hidden">
        <h1>Formulation 3</h1>
        <label for="slider3-1">IL/CHOL Pressure (PSI)</label>
        <input type="range" id="slider3-1" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('3')">
        <input type="number" id="slider3-1-value" class="slider-value" min="0" max="50" value="0" onchange="updateValues('3')"><br>
        <label for="slider3-2">PHOS/PEG Pressure (PSI)</label>
        <input type="range" id="slider3-2" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('3')">
        <input type="number" id="slider3-2-value" class="slider-value" min="0" max="50" value="0" onchange="updateValues('3')"><br>
        <div class="matrix-container">
            <table class="matrix-table">
                <thead>
                    <tr>
                        <th>IL</th>
                        <th>Phospholipid</th>
                        <th>Cholesterol</th>
                        <th>PEG</th>
                    </tr>
                </thead>
                <label for="matrix3-body">Composition Matrix (Molar Percent)</label>
                <tbody id="matrix3-body">
                    <tr>
                        <td id="3-row1-col1">0.00</td>
                        <td id="3-row1-col2">0.00</td>
                        <td id="3-row1-col3">0.00</td>
                        <td id="3-row1-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="3-row2-col1">0.00</td>
                        <td id="3-row2-col2">0.00</td>
                        <td id="3-row2-col3">0.00</td>
                        <td id="3-row2-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="3-row3-col1">0.00</td>
                        <td id="3-row3-col2">0.00</td>
                        <td id="3-row3-col3">0.00</td>
                        <td id="3-row3-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="3-row4-col1">0.00</td>
                        <td id="3-row4-col2">0.00</td>
                        <td id="3-row4-col3">0.00</td>
                        <td id="3-row4-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="3-row5-col1">0.00</td>
                        <td id="3-row5-col2">0.00</td>
                        <td id="3-row5-col3">0.00</td>
                        <td id="3-row5-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="3-row6-col1">0.00</td>
                        <td id="3-row6-col2">0.00</td>
                        <td id="3-row6-col3">0.00</td>
                        <td id="3-row6-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="3-row7-col1">0.00</td>
                        <td id="3-row7-col2">0.00</td>
                        <td id="3-row7-col3">0.00</td>
                        <td id="3-row7-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="3-row8-col1">0.00</td>
                        <td id="3-row8-col2">0.00</td>
                        <td id="3-row8-col3">0.00</td>
                        <td id="3-row8-col4">0.00</td>
                    </tr>
                </tbody>
            </table>
        </div>
        <button class="home_button" onclick="showMainPage()">Back to Main Page</button>
    </div>

    <div id="formulation4" class="container hidden">
        <h1>Formulation 4</h1>
        <label for="slider4-1">IL/CHOL Pressure (PSI)</label>
        <input type="range" id="slider4-1" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('4')">
        <input type="number" id="slider4-1-value" class="slider-value" min="0" max="50" value="0" onchange="updateValues('4')"><br>
        <label for="slider4-2">PHOS/PEG Pressure (PSI)</label>
        <input type="range" id="slider4-2" class="slider" min="0" max="50" value="0" step="0.1" oninput="updateValues('4')">
        <input type="number" id="slider4-2-value" class="slider-value" min="0" max="50" value="0" onchange="updateValues('4')"><br>
        <div class="matrix-container">
            <table class="matrix-table">
                <thead>
                    <tr>
                        <th>IL</th>
                        <th>Phospholipid</th>
                        <th>Cholesterol</th>
                        <th>PEG</th>
                    </tr>
                </thead>
                <label for="matrix4-body">Composition Matrix (Molar Percent)</label>
                <tbody id="matrix4-body">
                    <tr>
                        <td id="4-row1-col1">0.00</td>
                        <td id="4-row1-col2">0.00</td>
                        <td id="4-row1-col3">0.00</td>
                        <td id="4-row1-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="4-row2-col1">0.00</td>
                        <td id="4-row2-col2">0.00</td>
                        <td id="4-row2-col3">0.00</td>
                        <td id="4-row2-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="4-row3-col1">0.00</td>
                        <td id="4-row3-col2">0.00</td>
                        <td id="4-row3-col3">0.00</td>
                        <td id="4-row3-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="4-row4-col1">0.00</td>
                        <td id="4-row4-col2">0.00</td>
                        <td id="4-row4-col3">0.00</td>
                        <td id="4-row4-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="4-row5-col1">0.00</td>
                        <td id="4-row5-col2">0.00</td>
                        <td id="4-row5-col3">0.00</td>
                        <td id="4-row5-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="4-row6-col1">0.00</td>
                        <td id="4-row6-col2">0.00</td>
                        <td id="4-row6-col3">0.00</td>
                        <td id="4-row6-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="4-row7-col1">0.00</td>
                        <td id="4-row7-col2">0.00</td>
                        <td id="4-row7-col3">0.00</td>
                        <td id="4-row7-col4">0.00</td>
                    </tr>
                    <tr>
                        <td id="4-row8-col1">0.00</td>
                        <td id="4-row8-col2">0.00</td>
                        <td id="4-row8-col3">0.00</td>
                        <td id="4-row8-col4">0.00</td>
                    </tr>
                </tbody>
            </table>
        </div>
        <button class="home_button" onclick="showMainPage()">Back to Main Page</button>
    </div>

    <script>
    
    	let formulation1Values = [1, 1, 1, 1, 0, 0];
        let formulation2Values = [1, 1, 1, 1, 0, 0];
        let formulation3Values = [1, 1, 1, 1, 0, 0];
        let formulation4Values = [1, 1, 1, 1, 0, 0];
        let mw = [0,0,0,0];
        let Rt = [11031.616, 11031.616];
        let Rt_PHOS_PEG = 11031.616;
        let comp = [
            [0.085, 0.172268119], // Row 1
            [0.10, 0.158762881], // Row 2
            [0.11, 0.145257643], // Row 3
            [0.12, 0.131752405], // Row 4
            [0.13, 0.118247167], // Row 5
            [0.14, 0.104741928], // Row 6
            [0.15, 0.097989309], // Row 7
            [0.165, 0.070978833] // Row 8
        ];
        
        function updateValues(val) {
          formulation1Values = [
            document.getElementById('textbox1').value.trim(),
            document.getElementById('textbox2').value.trim(),
            document.getElementById('textbox3').value.trim(),
            document.getElementById('textbox4').value.trim(),
            document.getElementById('textbox5').value.trim(),
            document.getElementById('textbox6').value.trim(),
            document.getElementById('textbox7').value.trim(),
            document.getElementById('textbox8').value.trim(),
            document.getElementById('slider1-1').value,
            document.getElementById('slider1-2').value
          ];
          updateMatrix('formulation1');
          formulation2Values = [
            document.getElementById('textbox1').value.trim(),
            document.getElementById('textbox2').value.trim(),
            document.getElementById('textbox3').value.trim(),
            document.getElementById('textbox4').value.trim(),
            document.getElementById('textbox5').value.trim(),
            document.getElementById('textbox6').value.trim(),
            document.getElementById('textbox7').value.trim(),
            document.getElementById('textbox8').value.trim(),
            document.getElementById('slider2-1').value,
            document.getElementById('slider2-2').value
          ];
          updateMatrix('formulation2');
          formulation3Values = [
            document.getElementById('textbox1').value.trim(),
            document.getElementById('textbox2').value.trim(),
            document.getElementById('textbox3').value.trim(),
            document.getElementById('textbox4').value.trim(),
            document.getElementById('textbox5').value.trim(),
            document.getElementById('textbox6').value.trim(),
            document.getElementById('textbox7').value.trim(),
            document.getElementById('textbox8').value.trim(),
            document.getElementById('slider3-1').value,
            document.getElementById('slider3-2').value
          ];
          updateMatrix('formulation3');
          formulation4Values = [
            document.getElementById('textbox1').value.trim(),
            document.getElementById('textbox2').value.trim(),
            document.getElementById('textbox3').value.trim(),
            document.getElementById('textbox4').value.trim(),
            document.getElementById('textbox5').value.trim(),
            document.getElementById('textbox6').value.trim(),
            document.getElementById('textbox7').value.trim(),
            document.getElementById('textbox8').value.trim(),
            document.getElementById('slider4-1').value,
            document.getElementById('slider4-2').value
          ];
          switch(val){
          	case('1'):
          		updateMatrix('formulation1');
            case('2'):
          		updateMatrix('formulation2');
            case('3'):
          		updateMatrix('formulation3');
            case('4'):
          		updateMatrix('formulation4');
          }
        }

        function updateMatrix(formulationId) {
        	let conc = [
              0,
              0,
              0,
              0
            ];

          	// Calculate pressures
          	let P_IL_CHOL = 0;
          	let P_etOH = P_IL_CHOL;
         	let P_PHOS_PEG = 0;
        	switch (formulationId) {
                case 'formulation1':
                    // Calculate concentrations
                    conc = [
                        4 * parseFloat(formulation1Values[0]),
                        4 * parseFloat(formulation1Values[1]),
                        4 * parseFloat(formulation1Values[2]),
                        4 * parseFloat(formulation1Values[3])
                    ];
                    mw = [
                    	parseFloat(formulation1Values[4]),
                        parseFloat(formulation1Values[5]),
                        parseFloat(formulation1Values[6]),
                        parseFloat(formulation1Values[7])
                    ];
                    // Calculate pressures
                    P_IL_CHOL = formulation1Values[8] * 101325 / 14.696;
                    P_etOH = (40-formulation1Values[8])* 101325 / 14.696;
                    P_PHOS_PEG = formulation1Values[9] * 101325 / 14.696;
                    break;
                 case 'formulation2':
                 	conc = [
                        4 * parseFloat(formulation2Values[0]),
                        4 * parseFloat(formulation2Values[1]),
                        4 * parseFloat(formulation2Values[2]),
                        4 * parseFloat(formulation2Values[3])
                    ];
					mw = [
                    	parseFloat(formulation2Values[4]),
                        parseFloat(formulation2Values[5]),
                        parseFloat(formulation2Values[6]),
                        parseFloat(formulation2Values[7])
                    ];
                    // Calculate pressures
                    P_IL_CHOL = formulation2Values[8] * 101325 / 14.696;
                    P_etOH = (40-formulation2Values[8])* 101325 / 14.696;
                    P_PHOS_PEG = formulation2Values[9] * 101325 / 14.696;
                 	break;
                 case 'formulation3':
                 	conc = [
                        4 * parseFloat(formulation3Values[0]),
                        4 * parseFloat(formulation3Values[1]),
                        4 * parseFloat(formulation3Values[2]),
                        4 * parseFloat(formulation3Values[3])
                    ];
					mw = [
                    	parseFloat(formulation3Values[4]),
                        parseFloat(formulation3Values[5]),
                        parseFloat(formulation3Values[6]),
                        parseFloat(formulation3Values[7])
                    ];
                    // Calculate pressures
                    P_IL_CHOL = formulation3Values[8] * 101325 / 14.696;
                    P_etOH = (40-formulation3Values[8])* 101325 / 14.696;
                    P_PHOS_PEG = formulation3Values[9] * 101325 / 14.696;
                 	break;
                 case 'formulation4':
                 	conc = [
                        4 * parseFloat(formulation4Values[0]),
                        4 * parseFloat(formulation4Values[1]),
                        4 * parseFloat(formulation4Values[2]),
                        4 * parseFloat(formulation4Values[3])
                    ];
					mw = [
                    	parseFloat(formulation3Values[4]),
                        parseFloat(formulation3Values[5]),
                        parseFloat(formulation3Values[6]),
                        parseFloat(formulation3Values[7])
                    ];
                    // Calculate pressures
                    P_IL_CHOL = formulation4Values[8] * 101325 / 14.696;
                    P_etOH = (40-formulation4Values[8])* 101325 / 14.696;
                    P_PHOS_PEG = formulation4Values[9] * 101325 / 14.696;
                 	break;
                 default:
                    break;
                 }
                 
          // Calculate Qt values
          let Qt = [
            60 * P_IL_CHOL / Rt[0],
            60 * P_etOH / Rt[1]
          ];
          let Qt_PHOS_PEG = 60 * P_PHOS_PEG / Rt_PHOS_PEG;

          // Initialize flow rates array
          let flow_rates = [
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]
          ];

          // Calculate flow rates
          for (let i = 0; i < 2; i++) { // Iterate over Qt array indices (0 and 1)
            for (let j = 0; j < 8; j++) { // Iterate over comp array indices (0 to 7)
              flow_rates[j][i] = comp[j][0] * Qt[i];
            }
          }
          // Calculate flow rate for PHOS_PEG
          let flow_rate_PHOS_PEG = [];
          for (let i = 0; i < 8; i++) {
            flow_rate_PHOS_PEG[i] = comp[i][1] * Qt_PHOS_PEG;
          }
          // Concatenate flow_rates and flow_rate_PHOS_PEG into flow_rates_all_inputs
          let flow_rates_all_inputs = flow_rates.map((value, index) => [...value, flow_rate_PHOS_PEG[index]]);


          // Initialize mass flow rates arrays
          let mass_flow_rates_IL_CHOL = [
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0]
          ];
          let mass_flow_rates_PHOS_PEG = [
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0]
          ];

          // Calculate mass flow rates in ug/min
          for (let i = 0; i < 2; i++) {
            for (let j = 0; j < 8; j++) {
              mass_flow_rates_IL_CHOL[j][i] = flow_rates[j][i] * conc[2 * i];
              mass_flow_rates_PHOS_PEG[j][i] = flow_rate_PHOS_PEG[j] * conc[2 * i + 1];
            }
          }

          // Concatenate mass_flow_rates into mass_flow_rates array
          mass_flow_rates = [];
          for (let j = 0; j < 8; j++) { // Loop through 8 formulations
            mass_flow_rates.push([
              mass_flow_rates_IL_CHOL[j][0],  // IL_CHOL for formulation j
              mass_flow_rates_PHOS_PEG[j][0],  // PHOS_PEG for formulation j
              mass_flow_rates_IL_CHOL[j][1],
              mass_flow_rates_PHOS_PEG[j][1]
            ]);
          }
          // Initialize molar flow rates array
          let molar_flow_rates = [
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
          ];
          // Calculate molar flow rates in umoles/min
          for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 8; j++) {
              molar_flow_rates[j][i] = mass_flow_rates[j][i] / mw[i];
            }
          }
          // Calculate moles array
          let moles = [];
          for (let i = 0; i < 8; i++) {
            moles[i] = molar_flow_rates[i].reduce((acc, cur) => acc + cur, 0);
          }
          // Initialize molar percentages array
          let molar_percentages = [[0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                   [0,0,0,0],
                                  ];

          // Calculate molar percentages
          for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 8; j++) {
              molar_percentages[j][i] = (100 * molar_flow_rates[j][i] / moles[j]).toFixed(2);
            }
          }
          switch (formulationId) {
                case 'formulation1':
                    document.getElementById('1-row1-col1').innerHTML = molar_percentages[0][0];
                    document.getElementById('1-row1-col2').innerHTML = molar_percentages[0][1];
                    document.getElementById('1-row1-col3').innerHTML = molar_percentages[0][2];
                    document.getElementById('1-row1-col4').innerHTML = molar_percentages[0][3];
                    
                    document.getElementById('1-row2-col1').innerHTML = molar_percentages[1][0];
                    document.getElementById('1-row2-col2').innerHTML = molar_percentages[1][1];
                    document.getElementById('1-row2-col3').innerHTML = molar_percentages[1][2];
                    document.getElementById('1-row2-col4').innerHTML = molar_percentages[1][3];
                    
                    document.getElementById('1-row3-col1').innerHTML = molar_percentages[2][0];
                    document.getElementById('1-row3-col2').innerHTML = molar_percentages[2][1];
                    document.getElementById('1-row3-col3').innerHTML = molar_percentages[2][2];
                    document.getElementById('1-row3-col4').innerHTML = molar_percentages[2][3];
                    
                    document.getElementById('1-row4-col1').innerHTML = molar_percentages[3][0];
                    document.getElementById('1-row4-col2').innerHTML = molar_percentages[3][1];
                    document.getElementById('1-row4-col3').innerHTML = molar_percentages[3][2];
                    document.getElementById('1-row4-col4').innerHTML = molar_percentages[3][3];
                    
                    document.getElementById('1-row5-col1').innerHTML = molar_percentages[4][0];
                    document.getElementById('1-row5-col2').innerHTML = molar_percentages[4][1];
                    document.getElementById('1-row5-col3').innerHTML = molar_percentages[4][2];
                    document.getElementById('1-row5-col4').innerHTML = molar_percentages[4][3];
                    
                    document.getElementById('1-row6-col1').innerHTML = molar_percentages[5][0];
                    document.getElementById('1-row6-col2').innerHTML = molar_percentages[5][1];
                    document.getElementById('1-row6-col3').innerHTML = molar_percentages[5][2];
                    document.getElementById('1-row6-col4').innerHTML = molar_percentages[5][3];
                    
                    document.getElementById('1-row7-col1').innerHTML = molar_percentages[6][0];
                    document.getElementById('1-row7-col2').innerHTML = molar_percentages[6][1];
                    document.getElementById('1-row7-col3').innerHTML = molar_percentages[6][2];
                    document.getElementById('1-row7-col4').innerHTML = molar_percentages[6][3];
                    
                    document.getElementById('1-row8-col1').innerHTML = molar_percentages[7][0];
                    document.getElementById('1-row8-col2').innerHTML = molar_percentages[7][1];
                    document.getElementById('1-row8-col3').innerHTML = molar_percentages[7][2];
                    document.getElementById('1-row8-col4').innerHTML = molar_percentages[7][3];
                    break;
                 case 'formulation2':
                 	document.getElementById('2-row1-col1').innerHTML = molar_percentages[0][0];
                    document.getElementById('2-row1-col2').innerHTML = molar_percentages[0][1];
                    document.getElementById('2-row1-col3').innerHTML = molar_percentages[0][2];
                    document.getElementById('2-row1-col4').innerHTML = molar_percentages[0][3];
                    
                    document.getElementById('2-row2-col1').innerHTML = molar_percentages[1][0];
                    document.getElementById('2-row2-col2').innerHTML = molar_percentages[1][1];
                    document.getElementById('2-row2-col3').innerHTML = molar_percentages[1][2];
                    document.getElementById('2-row2-col4').innerHTML = molar_percentages[1][3];
                    
                    document.getElementById('2-row3-col1').innerHTML = molar_percentages[2][0];
                    document.getElementById('2-row3-col2').innerHTML = molar_percentages[2][1];
                    document.getElementById('2-row3-col3').innerHTML = molar_percentages[2][2];
                    document.getElementById('2-row3-col4').innerHTML = molar_percentages[2][3];
                   
                    document.getElementById('2-row4-col1').innerHTML = molar_percentages[3][0];
                    document.getElementById('2-row4-col2').innerHTML = molar_percentages[3][1];
                    document.getElementById('2-row4-col3').innerHTML = molar_percentages[3][2];
                    document.getElementById('2-row4-col4').innerHTML = molar_percentages[3][3];
                    
                    document.getElementById('2-row5-col1').innerHTML = molar_percentages[4][0];
                    document.getElementById('2-row5-col2').innerHTML = molar_percentages[4][1];
                    document.getElementById('2-row5-col3').innerHTML = molar_percentages[4][2];
                    document.getElementById('2-row5-col4').innerHTML = molar_percentages[4][3];
                    
                    document.getElementById('2-row6-col1').innerHTML = molar_percentages[5][0];
                    document.getElementById('2-row6-col2').innerHTML = molar_percentages[5][1];
                    document.getElementById('2-row6-col3').innerHTML = molar_percentages[5][2];
                    document.getElementById('2-row6-col4').innerHTML = molar_percentages[5][3];
                    
                    document.getElementById('2-row7-col1').innerHTML = molar_percentages[6][0];
                    document.getElementById('2-row7-col2').innerHTML = molar_percentages[6][1];
                    document.getElementById('2-row7-col3').innerHTML = molar_percentages[6][2];
                    document.getElementById('2-row7-col4').innerHTML = molar_percentages[6][3];
                    
                    document.getElementById('2-row8-col1').innerHTML = molar_percentages[7][0];
                    document.getElementById('2-row8-col2').innerHTML = molar_percentages[7][1];
                    document.getElementById('2-row8-col3').innerHTML = molar_percentages[7][2];
                    document.getElementById('2-row8-col4').innerHTML = molar_percentages[7][3];
                 	break;
                 case 'formulation3':
                 	document.getElementById('3-row1-col1').innerHTML = molar_percentages[0][0];
                    document.getElementById('3-row1-col2').innerHTML = molar_percentages[0][1];
                    document.getElementById('3-row1-col3').innerHTML = molar_percentages[0][2];
                    document.getElementById('3-row1-col4').innerHTML = molar_percentages[0][3];
                    
                    document.getElementById('3-row2-col1').innerHTML = molar_percentages[1][0];
                    document.getElementById('3-row2-col2').innerHTML = molar_percentages[1][1];
                    document.getElementById('3-row2-col3').innerHTML = molar_percentages[1][2];
                    document.getElementById('3-row2-col4').innerHTML = molar_percentages[1][3];
                    
                    document.getElementById('3-row3-col1').innerHTML = molar_percentages[2][0];
                    document.getElementById('3-row3-col2').innerHTML = molar_percentages[2][1];
                    document.getElementById('3-row3-col3').innerHTML = molar_percentages[2][2];
                    document.getElementById('3-row3-col4').innerHTML = molar_percentages[2][3];
                    
                    document.getElementById('3-row4-col1').innerHTML = molar_percentages[3][0];
                    document.getElementById('3-row4-col2').innerHTML = molar_percentages[3][1];
                    document.getElementById('3-row4-col3').innerHTML = molar_percentages[3][2];
                    document.getElementById('3-row4-col4').innerHTML = molar_percentages[3][3];
                    
                    document.getElementById('3-row5-col1').innerHTML = molar_percentages[4][0];
                    document.getElementById('3-row5-col2').innerHTML = molar_percentages[4][1];
                    document.getElementById('3-row5-col3').innerHTML = molar_percentages[4][2];
                    document.getElementById('3-row5-col4').innerHTML = molar_percentages[4][3];
                    
                    document.getElementById('3-row6-col1').innerHTML = molar_percentages[5][0];
                    document.getElementById('3-row6-col2').innerHTML = molar_percentages[5][1];
                    document.getElementById('3-row6-col3').innerHTML = molar_percentages[5][2];
                    document.getElementById('3-row6-col4').innerHTML = molar_percentages[5][3];
                    
                    document.getElementById('3-row7-col1').innerHTML = molar_percentages[6][0];
                    document.getElementById('3-row7-col2').innerHTML = molar_percentages[6][1];
                    document.getElementById('3-row7-col3').innerHTML = molar_percentages[6][2];
                    document.getElementById('3-row7-col4').innerHTML = molar_percentages[6][3];
                    
                    document.getElementById('3-row8-col1').innerHTML = molar_percentages[7][0];
                    document.getElementById('3-row8-col2').innerHTML = molar_percentages[7][1];
                    document.getElementById('3-row8-col3').innerHTML = molar_percentages[7][2];
                    document.getElementById('3-row8-col4').innerHTML = molar_percentages[7][3];
                 	break;
                 case 'formulation4':
                 	document.getElementById('4-row1-col1').innerHTML = molar_percentages[0][0];
                    document.getElementById('4-row1-col2').innerHTML = molar_percentages[0][1];
                    document.getElementById('4-row1-col3').innerHTML = molar_percentages[0][2];
                    document.getElementById('4-row1-col4').innerHTML = molar_percentages[0][3];
                    
                    document.getElementById('4-row2-col1').innerHTML = molar_percentages[1][0];
                    document.getElementById('4-row2-col2').innerHTML = molar_percentages[1][1];
                    document.getElementById('4-row2-col3').innerHTML = molar_percentages[1][2];
                    document.getElementById('4-row2-col4').innerHTML = molar_percentages[1][3];
                    
                    document.getElementById('4-row3-col1').innerHTML = molar_percentages[2][0];
                    document.getElementById('4-row3-col2').innerHTML = molar_percentages[2][1];
                    document.getElementById('4-row3-col3').innerHTML = molar_percentages[2][2];
                    document.getElementById('4-row3-col4').innerHTML = molar_percentages[2][3];
                    
                    document.getElementById('4-row4-col1').innerHTML = molar_percentages[3][0];
                    document.getElementById('4-row4-col2').innerHTML = molar_percentages[3][1];
                    document.getElementById('4-row4-col3').innerHTML = molar_percentages[3][2];
                    document.getElementById('4-row4-col4').innerHTML = molar_percentages[3][3];
                    
                    document.getElementById('4-row5-col1').innerHTML = molar_percentages[4][0];
                    document.getElementById('4-row5-col2').innerHTML = molar_percentages[4][1];
                    document.getElementById('4-row5-col3').innerHTML = molar_percentages[4][2];
                    document.getElementById('4-row5-col4').innerHTML = molar_percentages[4][3];
                    
                    document.getElementById('4-row6-col1').innerHTML = molar_percentages[5][0];
                    document.getElementById('4-row6-col2').innerHTML = molar_percentages[5][1];
                    document.getElementById('4-row6-col3').innerHTML = molar_percentages[5][2];
                    document.getElementById('4-row6-col4').innerHTML = molar_percentages[5][3];
                    
                    document.getElementById('4-row7-col1').innerHTML = molar_percentages[6][0];
                    document.getElementById('4-row7-col2').innerHTML = molar_percentages[6][1];
                    document.getElementById('4-row7-col3').innerHTML = molar_percentages[6][2];
                    document.getElementById('4-row7-col4').innerHTML = molar_percentages[6][3];
                    
                    document.getElementById('4-row8-col1').innerHTML = molar_percentages[7][0];
                    document.getElementById('4-row8-col2').innerHTML = molar_percentages[7][1];
                    document.getElementById('4-row8-col3').innerHTML = molar_percentages[7][2];
                    document.getElementById('4-row8-col4').innerHTML = molar_percentages[7][3];
                 	break;
                 default:
                    break;
                 }
                 
        }

        function showFormulation(formulationId) {
            document.getElementById('mainPage').classList.add('hidden');
            document.getElementById(formulationId).classList.remove('hidden');
        }

        function showMainPage() {
            document.getElementById('mainPage').classList.remove('hidden');
            document.getElementById('formulation1').classList.add('hidden');
            document.getElementById('formulation2').classList.add('hidden');
            document.getElementById('formulation3').classList.add('hidden');
            document.getElementById('formulation4').classList.add('hidden');
        }
        // Update slider value display when changed
        document.querySelectorAll('.slider').forEach(item => {
            const updateSliderValues = () => {
                const value = item.value;
                const id = item.id + '-value';
                const id_2 = 'pressure' + item.id.substring(6);
                
                // Update displayed values
                document.getElementById(id).value = value;
                document.getElementById(id_2).value = value;
            };
            
            // Input event listener (for immediate value update)
            item.addEventListener('input', () => {
                updateSliderValues();
            });
            
            // Change event listener (to send data when value is finalized)
            item.addEventListener('change', () => {
                updateSliderValues();
                
                const value = item.value;
                const id_2 = 'pressure' + item.id.substring(6);
                
                // Send data to server
                sendData(id_2, value);
            });
        });
        document.querySelectorAll('.slider-value').forEach(item => {
              item.addEventListener('input', function() {
                  const value = this.value;
                  const id = this.id.replace('-value', '');
                  const id_2 = 'pressure'+this.id.substring(6,9);
                  document.getElementById(id).value = value;
                  document.getElementById(id_2).value = value;
                  updateValues(id.split('-')[0].replace('slider', ''));
                  sendData(id_2, document.getElementById(id_2).value);
              });
        });
        document.querySelectorAll('.pressure-window').forEach(item => {
          item.addEventListener('input', function() {
              const value = this.value;
              const id = 'slider'+this.id.substring(8);
              const id_2 = 'slider'+this.id.substring(8)+'-value';
              document.getElementById(id).value = value;
              document.getElementById(id_2).value = value;
              updateValues(id.split('-')[0].replace('slider', ''));
              sendData(this.id, this.value);
          });
        });
        function sendData(id, value) {
            fetch(`/send?${id}=${value}`, { method: 'GET' })
                .then(response => response.text())
                .then(data => {
                    console.log(data);
                })
                .catch(error => {
                    console.error('Error:', error);
                });
        }
    </script>
</body>
</html>
)===";


WiFiServer server(80);  // Port 80 is standard for websites




void setup() {
  Serial.begin(115200);
  mySerial.begin(115200); // Initialize serial communication with 9600 baud rate
  WiFi.softAP(ssid, password);  // No password
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}


void loop() {
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        //Serial.println("new comm");
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.print(body);
            break;
          } else {
            // Parse the GET request
            if (currentLine.startsWith("GET /")) {
              int equalSignIndex = currentLine.indexOf('=');
              if (equalSignIndex != -1) {
                String id = currentLine.substring(10, equalSignIndex);
                Serial.println(id);
                float value = currentLine.substring(equalSignIndex + 1).toFloat();
                Serial.println(value);
                // Serial.println(value);
                // Serial.println(id);

                // Update the corresponding variable based on the ID
                // Fixed to use correct formulation variables (matching 20251031 structure)
                if (id == "pressure1-1") {
                  E1F1 = value;
                  E2F1 = 40-value;
                  A1F1 = E1F1*1.5;
                  A2F1 = E2F1*1.5;  // Fixed: was E2F2
                } else if (id == "pressure1-2") {
                  E3F1 = value;
                  A3F1 = E3F1*1.5;
                } else if (id == "pressure2-1") {
                  E1F2 = value;
                  E2F2 = 40-value;
                  A1F2 = E1F2*1.5;  // Fixed: was E1F1
                  A2F2 = E2F2*1.5;
                } else if (id == "pressure2-2") {
                  E3F2 = value;
                  A3F2 = E3F2*1.5;  // Fixed: was E3F1
                } else if (id == "pressure3-1") {
                  E1F3 = value;
                  E2F3 = 40-value;
                  A1F3 = E1F3*1.5;  // Fixed: was E1F1
                  A2F3 = E2F3*1.5;  // Fixed: was E2F2
                } else if (id == "pressure3-2") {
                  E3F3 = value;
                  A3F3 = E3F3*1.5;  // Fixed: was E3F1
                } else if (id == "pressure4-1") {
                  E1F4 = value;
                  E2F4 = 40-value;
                  A1F4 = E1F4*1.5;  // Fixed: was E1F1
                  A2F4 = E2F4*1.5;  // Fixed: was E2F2
                } else if (id == "pressure4-2") {
                  E3F4 = value;
                  A3F4 = E3F4*1.5;  // Fixed: was E3F1
                } else if (id == "formulationCountValue"){
                  // Updated to match 20251031: 9 regulators per formulation
                  // Format: AS, BS, CS, DS, ES, FS, GS, HS, IS (last 3 set to 0)
                  float dataArray[rows][cols] = {
                    {E1F1, E2F1, E3F1, A1F1, A2F1, A3F1, 0, 0, 0},
                    {E1F2, E2F2, E3F2, A1F2, A2F2, A3F2, 0, 0, 0},
                    {E1F3, E2F3, E3F3, A1F3, A2F3, A3F3, 0, 0, 0},
                    {E1F4, E2F4, E3F4, A1F4, A2F4, A3F4, 0, 0, 0}
                  };
                  // Serial.println(E1F1);
                  for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols; j++) {
                      mySerial.print(dataArray[i][j]);
                      if (j < cols - 1) {
                        mySerial.print(","); // Separate elements in a row with a comma
                        delay(20);
                      }
                    }
                    mySerial.print("f"); // Send new line to indicate completion of a row
                  }
                }
                else if (id == "terminate"){
                  mySerial.print("t"); // termination character
                }
              }
            }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }  
      }
    }
    client.stop();
  }
}