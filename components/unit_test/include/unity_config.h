// Unity configuration for custom output handling
#pragma once

// Route Unity's character output through our filter implementation
int capture_unity_char(int c);
#define UNITY_OUTPUT_CHAR capture_unity_char

