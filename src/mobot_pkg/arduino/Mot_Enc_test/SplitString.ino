void splitSerialInput() {
  String inputString = Serial.readStringUntil('\n'); // Read input until newline character
  inputString.trim(); // Remove leading/trailing whitespaces
  
  const int maxSplitParts = 10; // Maximum number of parts to split
  String splitParts[maxSplitParts]; // Array to hold split parts
  int splitCount = 0; // Counter for split parts
  
  int lastIndex = 0; // Index to keep track of the splitting process
  
  // Loop through the input string to split it by comma
  while (splitCount < maxSplitParts - 1) {
    int commaIndex = inputString.indexOf(',', lastIndex); // Find comma position
    
    if (commaIndex >= 0) { // If a comma is found
      splitParts[splitCount] = inputString.substring(lastIndex, commaIndex); // Extract substring between last index and comma
      lastIndex = commaIndex + 1; // Update last index to start from the next character after comma
      splitCount++; // Increment the split counter
    } else {
      break; // Break the loop if no more commas are found
    }
  }
  
  // Capture the last segment after the last comma
  if (lastIndex < inputString.length() && splitCount < maxSplitParts) {
    splitParts[splitCount] = inputString.substring(lastIndex); // Extract the substring after the last comma
    splitCount++; // Increment the split counter
  }
  
  // Printing the split parts
  Serial.println("Split parts:");
  for (int i = 0; i < splitCount; i++) {
    Serial.print("Part ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(splitParts[i]);
  }
}
