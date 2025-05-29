# Archive Build Fixes

This document describes the fixes applied to resolve errors when archiving the ME400 Control app.

## Issues Fixed

### 1. Preview Code Not Wrapped in DEBUG
**Problem**: Preview code using `ControlCoordinatorFactory.createMock()` was being included in release builds, causing "has no member 'createMock'" errors.

**Solution**: Wrapped all preview code in `#if DEBUG` blocks:
- `NContentView.swift` - Line 373-378
- `NSettingsView.swift` - Line 183-188

### 2. Sendable Conformance Issues
**Problem**: `ControlCoordinator` was being captured in `@Sendable` closures, causing concurrency errors.

**Solution**: 
- Marked `ControlCoordinator` class with `@MainActor` to ensure thread safety
- Updated Timer closure in `startQueryTimer()` to not capture self weakly in the Timer closure itself
- Marked factory methods in `ControlCoordinatorFactory` with `@MainActor`
- Marked `NME400App` struct with `@MainActor` to allow calling MainActor factory methods

### 3. Filtered Dot Size
**Additional Change**: Reduced the filtered dot size from 20x20 to 10x10 pixels in `NCameraView.swift` as requested.

## Files Modified
1. `NContentView.swift` - Added `#if DEBUG` wrapper for preview
2. `NSettingsView.swift` - Added `#if DEBUG` wrapper for preview
3. `NControlCoordinator.swift` - Added `@MainActor` to class and factory methods, fixed Timer closure
4. `NME400App.swift` - Added `@MainActor` to app struct
5. `NCameraView.swift` - Reduced filtered dot size

## Boundary Value Information
The camera view shows arrows when elements exceed the ±0.4 boundary in the normalized coordinate system. The actual coordinate values are displayed in the arrow indicators, showing where the element would be if the view extended beyond the boundary. 