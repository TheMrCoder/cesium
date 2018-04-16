define([
        '../Core/arraySlice',
        '../Core/Cartesian2',
        '../Core/Cartesian3',
        '../Core/Cartesian4',
        '../Core/Color',
        '../Core/combine',
        '../Core/ComponentDatatype',
        '../Core/defaultValue',
        '../Core/defined',
        '../Core/defineProperties',
        '../Core/destroyObject',
        '../Core/DeveloperError',
        '../Core/FeatureDetection',
        '../Core/getStringFromTypedArray',
        '../Core/Math',
        '../Core/Matrix3',
        '../Core/Matrix4',
        '../Core/oneTimeWarning',
        '../Core/OrthographicFrustum',
        '../Core/Plane',
        '../Core/PrimitiveType',
        '../Core/RuntimeError',
        '../Core/TaskProcessor',
        '../Core/Transforms',
        '../Renderer/Buffer',
        '../Renderer/BufferUsage',
        '../Renderer/DrawCommand',
        '../Renderer/Pass',
        '../Renderer/RenderState',
        '../Renderer/ShaderProgram',
        '../Renderer/ShaderSource',
        '../Renderer/VertexArray',
        '../ThirdParty/when',
        './BlendingState',
        './Cesium3DTileBatchTable',
        './Cesium3DTileFeature',
        './Cesium3DTileFeatureTable',
        './ClippingPlaneCollection',
        './getClipAndStyleCode',
        './getClippingFunction',
        './PointCloudEyeDomeLighting',
        './PointCloudShading',
        './SceneMode',
        './ShadowMode'
    ], function(
        arraySlice,
        Cartesian2,
        Cartesian3,
        Cartesian4,
        Color,
        combine,
        ComponentDatatype,
        defaultValue,
        defined,
        defineProperties,
        destroyObject,
        DeveloperError,
        FeatureDetection,
        getStringFromTypedArray,
        CesiumMath,
        Matrix3,
        Matrix4,
        oneTimeWarning,
        OrthographicFrustum,
        Plane,
        PrimitiveType,
        RuntimeError,
        TaskProcessor,
        Transforms,
        Buffer,
        BufferUsage,
        DrawCommand,
        Pass,
        RenderState,
        ShaderProgram,
        ShaderSource,
        VertexArray,
        when,
        BlendingState,
        Cesium3DTileBatchTable,
        Cesium3DTileFeature,
        Cesium3DTileFeatureTable,
        ClippingPlaneCollection,
        getClipAndStyleCode,
        getClippingFunction,
        PointCloudEyeDomeLighting,
        PointCloudShading,
        SceneMode,
        ShadowMode) {
    'use strict';

    // Bail out if the browser doesn't support typed arrays, to prevent the setup function
    // from failing, since we won't be able to create a WebGL context anyway.
    if (!FeatureDetection.supportsTypedArrays()) {
        return {};
    }

    var DecodingState = {
        NEEDS_DECODE : 0,
        DECODING : 1,
        READY : 2,
        FAILED : 3
    };

    function PointCloudStream(options) {
        this.pointCloudShading = new PointCloudShading(options.pointCloudShading);
        this.style = options.style;
        this.index = 0;
        this._pointCloudEyeDomeLighting = new PointCloudEyeDomeLighting();
        this._frames = [];
    }

    PointCloudStream.prototype.update = function(frameState) {
        var commandList = frameState.commandList;
        var lengthBeforeUpdate = commandList.length;

        var framesLength = this._frames.length;
        if (framesLength === 0) {
            return;
        }

        var index = this.index;
        if (index < 0 || index > framesLength) {
            // Not a valid index. Could throw an error instead.
            return;
        }

        // var frames = this._frames;
        // var length = frames.length;
        // for (var i = 0; i < length; ++i) {
        //     var frame = frames[i];
        //     frame.update(frameState);
        // }

        var frame = this._frames[index];
        frame.update(frameState);

        var lengthAfterUpdate = commandList.length;
        var addedCommandsLength = lengthAfterUpdate - lengthBeforeUpdate;

        if (this.pointCloudShading.attenuation &&
            this.pointCloudShading.eyeDomeLighting &&
            (addedCommandsLength > 0)) {
            this._pointCloudEyeDomeLighting.update(frameState, lengthBeforeUpdate, this.pointCloudShading);
        }
    };

    PointCloudStream.prototype.addFrame = function(arrayBuffer, modelMatrix) {
        var frame = new PointCloudFrame(this, modelMatrix, arrayBuffer, 0);
        this._frames.push(frame);
    };

    PointCloudStream.prototype.isDestroyed = function() {
        return false;
    };

    PointCloudStream.prototype.destroy = function() {
        return destroyObject(this);
    };

    /**
     * @private
     */
    function PointCloudFrame(stream, modelMatrix, arrayBuffer, byteOffset) {
        this._stream = stream;
        this._style = undefined;

        // Hold onto the payload until the render resources are created
        this._parsedContent = undefined;

        this._drawCommand = undefined;
        this._pickCommand = undefined;
        this._pickId = undefined; // Only defined when batchTable is undefined
        this._isTranslucent = false;
        this._styleTranslucent = false;
        this._constantColor = Color.clone(Color.WHITE);
        this._rtcCenter = undefined;

        // These values are used to regenerate the shader when the style changes
        this._styleableShaderAttributes = undefined;
        this._isQuantized = false;
        this._isOctEncoded16P = false;
        this._isRGB565 = false;
        this._hasColors = false;
        this._hasNormals = false;
        this._hasBatchIds = false;

        // Draco
        this._decodingState = DecodingState.READY;
        this._dequantizeInShader = false;
        this._isQuantizedDraco = false;
        this._isOctEncodedDraco = false;
        this._octEncodedRange = 0.0;

        // Use per-point normals to hide back-facing points.
        this.backFaceCulling = false;
        this._backFaceCulling = false;

        this._opaqueRenderState = undefined;
        this._translucentRenderState = undefined;

        this._highlightColor = Color.clone(Color.WHITE);
        this._pointSize = 1.0;
        this._quantizedVolumeScale = undefined;
        this._quantizedVolumeOffset = undefined;

        this.modelMatrix = Matrix4.clone(modelMatrix);
        this._modelMatrix = Matrix4.clone(modelMatrix);

        this._pointsLength = 0;
        this._geometryByteLength = 0;

        // Options for geometric error based attenuation
        this._attenuation = false;
        this._geometricErrorScale = undefined;
        this._maximumAttenuation = undefined;
        this._baseResolution = undefined;
        this._baseResolutionApproximation = undefined;

        initialize(this, arrayBuffer, byteOffset);
    }

    var sizeOfUint32 = Uint32Array.BYTES_PER_ELEMENT;

    function initialize(content, arrayBuffer, byteOffset) {
        byteOffset = defaultValue(byteOffset, 0);

        var uint8Array = new Uint8Array(arrayBuffer);
        var view = new DataView(arrayBuffer);
        byteOffset += sizeOfUint32;  // Skip magic

        var version = view.getUint32(byteOffset, true);
        if (version !== 1) {
            throw new RuntimeError('Only Point Cloud tile version 1 is supported.  Version ' + version + ' is not.');
        }
        byteOffset += sizeOfUint32;

        // Skip byteLength
        byteOffset += sizeOfUint32;

        var featureTableJsonByteLength = view.getUint32(byteOffset, true);
        if (featureTableJsonByteLength === 0) {
            throw new RuntimeError('Feature table must have a byte length greater than zero');
        }
        byteOffset += sizeOfUint32;

        var featureTableBinaryByteLength = view.getUint32(byteOffset, true);
        byteOffset += sizeOfUint32;

        var batchTableJsonByteLength = view.getUint32(byteOffset, true);
        byteOffset += sizeOfUint32;
        var batchTableBinaryByteLength = view.getUint32(byteOffset, true);
        byteOffset += sizeOfUint32;

        var featureTableString = getStringFromTypedArray(uint8Array, byteOffset, featureTableJsonByteLength);
        var featureTableJson = JSON.parse(featureTableString);
        byteOffset += featureTableJsonByteLength;

        var featureTableBinary = new Uint8Array(arrayBuffer, byteOffset, featureTableBinaryByteLength);
        byteOffset += featureTableBinaryByteLength;

        // Get the batch table JSON and binary
        var batchTableJson;
        var batchTableBinary;
        if (batchTableJsonByteLength > 0) {
            // Has a batch table JSON
            var batchTableString = getStringFromTypedArray(uint8Array, byteOffset, batchTableJsonByteLength);
            batchTableJson = JSON.parse(batchTableString);
            byteOffset += batchTableJsonByteLength;

            if (batchTableBinaryByteLength > 0) {
                // Has a batch table binary
                batchTableBinary = new Uint8Array(arrayBuffer, byteOffset, batchTableBinaryByteLength);
                byteOffset += batchTableBinaryByteLength;
            }
        }

        var featureTable = new Cesium3DTileFeatureTable(featureTableJson, featureTableBinary);

        var pointsLength = featureTable.getGlobalProperty('POINTS_LENGTH');
        featureTable.featuresLength = pointsLength;

        if (!defined(pointsLength)) {
            throw new RuntimeError('Feature table global property: POINTS_LENGTH must be defined');
        }

        // Get the positions
        var positions;
        var isQuantized = false;

        if (defined(featureTableJson.POSITION)) {
            positions = featureTable.getPropertyArray('POSITION', ComponentDatatype.FLOAT, 3);
            var rtcCenter = featureTable.getGlobalProperty('RTC_CENTER', ComponentDatatype.FLOAT, 3);
            if (defined(rtcCenter)) {
                content._rtcCenter = Cartesian3.unpack(rtcCenter);
            }
        } else if (defined(featureTableJson.POSITION_QUANTIZED)) {
            positions = featureTable.getPropertyArray('POSITION_QUANTIZED', ComponentDatatype.UNSIGNED_SHORT, 3);
            isQuantized = true;

            var quantizedVolumeScale = featureTable.getGlobalProperty('QUANTIZED_VOLUME_SCALE', ComponentDatatype.FLOAT, 3);
            if (!defined(quantizedVolumeScale)) {
                throw new RuntimeError('Global property: QUANTIZED_VOLUME_SCALE must be defined for quantized positions.');
            }
            content._quantizedVolumeScale = Cartesian3.unpack(quantizedVolumeScale);

            var quantizedVolumeOffset = featureTable.getGlobalProperty('QUANTIZED_VOLUME_OFFSET', ComponentDatatype.FLOAT, 3);
            if (!defined(quantizedVolumeOffset)) {
                throw new RuntimeError('Global property: QUANTIZED_VOLUME_OFFSET must be defined for quantized positions.');
            }
            content._quantizedVolumeOffset = Cartesian3.unpack(quantizedVolumeOffset);
        }

        // Get the colors
        var colors;
        var isTranslucent = false;
        var isRGB565 = false;

        if (defined(featureTableJson.RGBA)) {
            colors = featureTable.getPropertyArray('RGBA', ComponentDatatype.UNSIGNED_BYTE, 4);
            isTranslucent = true;
        } else if (defined(featureTableJson.RGB)) {
            colors = featureTable.getPropertyArray('RGB', ComponentDatatype.UNSIGNED_BYTE, 3);
        } else if (defined(featureTableJson.RGB565)) {
            colors = featureTable.getPropertyArray('RGB565', ComponentDatatype.UNSIGNED_SHORT, 1);
            isRGB565 = true;
        } else if (defined(featureTableJson.CONSTANT_RGBA)) {
            var constantRGBA  = featureTable.getGlobalProperty('CONSTANT_RGBA', ComponentDatatype.UNSIGNED_BYTE, 4);
            content._constantColor = Color.fromBytes(constantRGBA[0], constantRGBA[1], constantRGBA[2], constantRGBA[3], content._constantColor);
        } else {
            // Use a default constant color
            content._constantColor = Color.clone(Color.DARKGRAY, content._constantColor);
        }

        // Get the normals
        var normals;
        var isOctEncoded16P = false;

        if (defined(featureTableJson.NORMAL)) {
            normals = featureTable.getPropertyArray('NORMAL', ComponentDatatype.FLOAT, 3);
        } else if (defined(featureTableJson.NORMAL_OCT16P)) {
            normals = featureTable.getPropertyArray('NORMAL_OCT16P', ComponentDatatype.UNSIGNED_BYTE, 2);
            isOctEncoded16P = true;
        }

        // Get the batchIds and batch table. BATCH_ID does not need to be defined when the point cloud has per-point properties.
        var batchIds;
        if (defined(featureTableJson.BATCH_ID)) {
            batchIds = featureTable.getPropertyArray('BATCH_ID', ComponentDatatype.UNSIGNED_SHORT, 1);
        }

        var hasPositions = defined(positions);
        var hasColors = defined(colors);
        var hasNormals = defined(normals);
        var hasBatchIds = defined(batchIds);

        // Get the draco buffer and semantics
        var draco = featureTableJson.DRACO;
        var dracoBuffer;
        var dracoSemantics;
        var isQuantizedDraco = false;
        var isOctEncodedDraco = false;
        if (defined(draco)) {
            dracoSemantics = draco.semantics;
            var dracoByteOffset = draco.byteOffset;
            var dracoByteLength = draco.byteLength;
            if (!defined(dracoSemantics) || !defined(dracoByteOffset) || !defined(dracoByteLength)) {
                throw new RuntimeError('DRACO.semantics, DRACO.byteOffset, and DRACO.byteLength must be defined');
            }

            var dracoHasPositions = dracoSemantics.indexOf('POSITION') >= 0;
            var dracoHasRGB = dracoSemantics.indexOf('RGB') >= 0;
            var dracoHasRGBA = dracoSemantics.indexOf('RGBA') >= 0;
            var dracoHasColors = dracoHasRGB || dracoHasRGBA;
            var dracoHasNormals = dracoSemantics.indexOf('NORMAL') >= 0;
            var dracoHasBatchIds = dracoSemantics.indexOf('BATCH_ID') >= 0;
            dracoBuffer = arraySlice(featureTableBinary, dracoByteOffset, dracoByteOffset + dracoByteLength);

            if (dracoHasPositions) {
                isQuantized = false;
                isQuantizedDraco = content._dequantizeInShader;
                hasPositions = true;
            }
            if (dracoHasRGBA) {
                isTranslucent = true;
            } else if (dracoHasRGB) {
                isTranslucent = false;
            }
            if (dracoHasColors) {
                isRGB565 = false;
                hasColors = true;
            }
            if (dracoHasNormals) {
                isOctEncoded16P = false;
                isOctEncodedDraco = content._dequantizeInShader;
                hasNormals = true;
            }
            if (dracoHasBatchIds) {
                hasBatchIds = true;
            }

            content._decodingState = DecodingState.NEEDS_DECODE;
        }

        if (!hasPositions) {
            throw new RuntimeError('Either POSITION or POSITION_QUANTIZED must be defined.');
        }

        if (hasBatchIds) {
            throw new RuntimeError('PointCloudStream frame content cannot have BATCH_ID');
        }

        // If points are not batched and there are per-point properties, use these properties for styling purposes
        var styleableProperties;
        if (!hasBatchIds && defined(batchTableBinary)) {
            styleableProperties = Cesium3DTileBatchTable.getBinaryProperties(pointsLength, batchTableJson, batchTableBinary);

            // WebGL does not support UNSIGNED_INT, INT, or DOUBLE vertex attributes. Convert these to FLOAT.
            for (var name in styleableProperties) {
                if (styleableProperties.hasOwnProperty(name)) {
                    var property = styleableProperties[name];
                    var typedArray = property.typedArray;
                    var componentDatatype = ComponentDatatype.fromTypedArray(typedArray);
                    if (componentDatatype === ComponentDatatype.INT || componentDatatype === ComponentDatatype.UNSIGNED_INT || componentDatatype === ComponentDatatype.DOUBLE) {
                        oneTimeWarning('Cast pnts property to floats', 'Point cloud property "' + name + '" will be casted to a float array because INT, UNSIGNED_INT, and DOUBLE are not valid WebGL vertex attribute types. Some precision may be lost.');
                        property.typedArray = new Float32Array(typedArray);
                    }
                }
            }
        }

        content._parsedContent = {
            positions : positions,
            colors : colors,
            normals : normals,
            batchIds : batchIds,
            styleableProperties : styleableProperties,
            draco : {
                buffer : dracoBuffer,
                semantics : dracoSemantics,
                dequantizeInShader : content._dequantizeInShader
            }
        };
        content._pointsLength = pointsLength;
        content._isQuantized = isQuantized;
        content._isQuantizedDraco = isQuantizedDraco;
        content._isOctEncoded16P = isOctEncoded16P;
        content._isOctEncodedDraco = isOctEncodedDraco;
        content._isRGB565 = isRGB565;
        content._isTranslucent = isTranslucent;
        content._hasColors = hasColors;
        content._hasNormals = hasNormals;
        content._hasBatchIds = hasBatchIds;
    }

    var scratchPointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier = new Cartesian4();
    var scratchQuantizedVolumeScaleAndOctEncodedRange = new Cartesian4();

    var positionLocation = 0;
    var colorLocation = 1;
    var normalLocation = 2;
    var batchIdLocation = 3;
    var numberOfAttributes = 4;

    function createResources(content, frameState) {
        var context = frameState.context;
        var parsedContent = content._parsedContent;
        var pointsLength = content._pointsLength;
        var positions = parsedContent.positions;
        var colors = parsedContent.colors;
        var normals = parsedContent.normals;
        var batchIds = parsedContent.batchIds;
        var styleableProperties = parsedContent.styleableProperties;
        var hasStyleableProperties = defined(styleableProperties);
        var isQuantized = content._isQuantized;
        var isQuantizedDraco = content._isQuantizedDraco;
        var isOctEncoded16P = content._isOctEncoded16P;
        var isOctEncodedDraco = content._isOctEncodedDraco;
        var isRGB565 = content._isRGB565;
        var isTranslucent = content._isTranslucent;
        var hasColors = content._hasColors;
        var hasNormals = content._hasNormals;
        var hasBatchIds = content._hasBatchIds;

        var batchTable = content._batchTable;
        var hasBatchTable = defined(batchTable);

        var styleableVertexAttributes = [];
        var styleableShaderAttributes = {};
        content._styleableShaderAttributes = styleableShaderAttributes;

        if (hasStyleableProperties) {
            var attributeLocation = numberOfAttributes;

            for (var name in styleableProperties) {
                if (styleableProperties.hasOwnProperty(name)) {
                    var property = styleableProperties[name];
                    var typedArray = property.typedArray;
                    var componentCount = property.componentCount;
                    var componentDatatype = ComponentDatatype.fromTypedArray(typedArray);

                    var vertexBuffer = Buffer.createVertexBuffer({
                        context : context,
                        typedArray : property.typedArray,
                        usage : BufferUsage.STATIC_DRAW
                    });

                    content._geometryByteLength += vertexBuffer.sizeInBytes;

                    var vertexAttribute = {
                        index : attributeLocation,
                        vertexBuffer : vertexBuffer,
                        componentsPerAttribute : componentCount,
                        componentDatatype : componentDatatype,
                        normalize : false,
                        offsetInBytes : 0,
                        strideInBytes : 0
                    };

                    styleableVertexAttributes.push(vertexAttribute);
                    styleableShaderAttributes[name] = {
                        location : attributeLocation,
                        componentCount : componentCount
                    };
                    ++attributeLocation;
                }
            }
        }
        var uniformMap = {
            u_pointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier : function() {
                var scratch = scratchPointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier;
                scratch.x = content._attenuation ? content._maximumAttenuation : content._pointSize;
                scratch.y = 0.0;

                if (content._attenuation) {
                    var geometricError = content._baseResolution;
                    var depthMultiplier = context.drawingBufferHeight / frameState.camera.frustum.sseDenominator;

                    scratch.z = geometricError * content._geometricErrorScale;
                    scratch.w = depthMultiplier;
                }

                return scratch;
            },
            u_highlightColor : function() {
                return content._highlightColor;
            },
            u_constantColor : function() {
                return content._constantColor;
            }
        };

        if (isQuantized || isQuantizedDraco || isOctEncodedDraco) {
            uniformMap = combine(uniformMap, {
                u_quantizedVolumeScaleAndOctEncodedRange : function() {
                    var scratch = scratchQuantizedVolumeScaleAndOctEncodedRange;
                    if (defined(content._quantizedVolumeScale)) {
                        scratch.x = content._quantizedVolumeScale.x;
                        scratch.y = content._quantizedVolumeScale.y;
                        scratch.z = content._quantizedVolumeScale.z;
                    }
                    scratch.w = content._octEncodedRange;
                    return scratch;
                }
            });
        }

        var positionsVertexBuffer = Buffer.createVertexBuffer({
            context : context,
            typedArray : positions,
            usage : BufferUsage.STATIC_DRAW
        });
        content._geometryByteLength += positionsVertexBuffer.sizeInBytes;

        var colorsVertexBuffer;
        if (hasColors) {
            colorsVertexBuffer = Buffer.createVertexBuffer({
                context : context,
                typedArray : colors,
                usage : BufferUsage.STATIC_DRAW
            });
            content._geometryByteLength += colorsVertexBuffer.sizeInBytes;
        }

        var normalsVertexBuffer;
        if (hasNormals) {
            normalsVertexBuffer = Buffer.createVertexBuffer({
                context : context,
                typedArray : normals,
                usage : BufferUsage.STATIC_DRAW
            });
            content._geometryByteLength += normalsVertexBuffer.sizeInBytes;
        }

        var batchIdsVertexBuffer;
        if (hasBatchIds) {
            batchIdsVertexBuffer = Buffer.createVertexBuffer({
                context : context,
                typedArray : batchIds,
                usage : BufferUsage.STATIC_DRAW
            });
            content._geometryByteLength += batchIdsVertexBuffer.sizeInBytes;
        }

        var attributes = [];
        if (isQuantized) {
            attributes.push({
                index : positionLocation,
                vertexBuffer : positionsVertexBuffer,
                componentsPerAttribute : 3,
                componentDatatype : ComponentDatatype.UNSIGNED_SHORT,
                normalize : true, // Convert position to 0 to 1 before entering the shader
                offsetInBytes : 0,
                strideInBytes : 0
            });
        } else if (isQuantizedDraco) {
            attributes.push({
                index : positionLocation,
                vertexBuffer : positionsVertexBuffer,
                componentsPerAttribute : 3,
                componentDatatype : ComponentDatatype.UNSIGNED_SHORT,
                normalize : false, // Normalization is done in the shader based on quantizationBits
                offsetInBytes : 0,
                strideInBytes : 0
            });
        } else {
            attributes.push({
                index : positionLocation,
                vertexBuffer : positionsVertexBuffer,
                componentsPerAttribute : 3,
                componentDatatype : ComponentDatatype.FLOAT,
                normalize : false,
                offsetInBytes : 0,
                strideInBytes : 0
            });
        }

        if (hasColors) {
            if (isRGB565) {
                attributes.push({
                    index : colorLocation,
                    vertexBuffer : colorsVertexBuffer,
                    componentsPerAttribute : 1,
                    componentDatatype : ComponentDatatype.UNSIGNED_SHORT,
                    normalize : false,
                    offsetInBytes : 0,
                    strideInBytes : 0
                });
            } else {
                var colorComponentsPerAttribute = isTranslucent ? 4 : 3;
                attributes.push({
                    index : colorLocation,
                    vertexBuffer : colorsVertexBuffer,
                    componentsPerAttribute : colorComponentsPerAttribute,
                    componentDatatype : ComponentDatatype.UNSIGNED_BYTE,
                    normalize : true,
                    offsetInBytes : 0,
                    strideInBytes : 0
                });
            }
        }

        if (hasNormals) {
            if (isOctEncoded16P) {
                attributes.push({
                    index : normalLocation,
                    vertexBuffer : normalsVertexBuffer,
                    componentsPerAttribute : 2,
                    componentDatatype : ComponentDatatype.UNSIGNED_BYTE,
                    normalize : false,
                    offsetInBytes : 0,
                    strideInBytes : 0
                });
            } else if (isOctEncodedDraco) {
                // TODO : depending on the quantizationBits we could probably use BYTE
                attributes.push({
                    index : normalLocation,
                    vertexBuffer : normalsVertexBuffer,
                    componentsPerAttribute : 2,
                    componentDatatype : ComponentDatatype.UNSIGNED_SHORT,
                    normalize : false,
                    offsetInBytes : 0,
                    strideInBytes : 0
                });
            } else {
                attributes.push({
                    index : normalLocation,
                    vertexBuffer : normalsVertexBuffer,
                    componentsPerAttribute : 3,
                    componentDatatype : ComponentDatatype.FLOAT,
                    normalize : false,
                    offsetInBytes : 0,
                    strideInBytes : 0
                });
            }
        }

        if (hasBatchIds) {
            attributes.push({
                index : batchIdLocation,
                vertexBuffer : batchIdsVertexBuffer,
                componentsPerAttribute : 1,
                componentDatatype : ComponentDatatype.fromTypedArray(batchIds),
                normalize : false,
                offsetInBytes : 0,
                strideInBytes : 0
            });
        }

        if (hasStyleableProperties) {
            attributes = attributes.concat(styleableVertexAttributes);
        }

        var vertexArray = new VertexArray({
            context : context,
            attributes : attributes
        });

        var drawUniformMap = uniformMap;

        if (hasBatchTable) {
            drawUniformMap = batchTable.getUniformMapCallback()(uniformMap);
        }

        var pickUniformMap;

        if (hasBatchTable) {
            pickUniformMap = batchTable.getPickUniformMapCallback()(uniformMap);
        } else {
            content._pickId = context.createPickId({
                primitive : content._stream,
                content : content
            });

            pickUniformMap = combine(uniformMap, {
                czm_pickColor : function() {
                    return content._pickId.color;
                }
            });
        }

        content._opaqueRenderState = RenderState.fromCache({
            depthTest : {
                enabled : true
            }
        });

        content._translucentRenderState = RenderState.fromCache({
            depthTest : {
                enabled : true
            },
            depthMask : false,
            blending : BlendingState.ALPHA_BLEND
        });

        content._drawCommand = new DrawCommand({
            boundingVolume : undefined,
            cull : false,
            modelMatrix : new Matrix4(),
            primitiveType : PrimitiveType.POINTS,
            vertexArray : vertexArray,
            count : pointsLength,
            shaderProgram : undefined, // Updated in createShaders
            uniformMap : drawUniformMap,
            renderState : isTranslucent ? content._translucentRenderState : content._opaqueRenderState,
            pass : isTranslucent ? Pass.TRANSLUCENT : Pass.CESIUM_3D_TILE,
            owner : content,
            castShadows : false,
            receiveShadows : false
        });

        content._pickCommand = new DrawCommand({
            boundingVolume : undefined,
            cull : false,
            modelMatrix : new Matrix4(),
            primitiveType : PrimitiveType.POINTS,
            vertexArray : vertexArray,
            count : pointsLength,
            shaderProgram : undefined, // Updated in createShaders
            uniformMap : pickUniformMap,
            renderState : isTranslucent ? content._translucentRenderState : content._opaqueRenderState,
            pass : isTranslucent ? Pass.TRANSLUCENT : Pass.CESIUM_3D_TILE,
            owner : content
        });
    }

    var defaultProperties = ['POSITION', 'COLOR', 'NORMAL', 'POSITION_ABSOLUTE'];

    function getStyleableProperties(source, properties) {
        // Get all the properties used by this style
        var regex = /czm_tiles3d_style_(\w+)/g;
        var matches = regex.exec(source);
        while (matches !== null) {
            var name = matches[1];
            if (properties.indexOf(name) === -1) {
                properties.push(name);
            }
            matches = regex.exec(source);
        }
    }

    function getVertexAttribute(vertexArray, index) {
        var numberOfAttributes = vertexArray.numberOfAttributes;
        for (var i = 0; i < numberOfAttributes; ++i) {
            var attribute = vertexArray.getAttribute(i);
            if (attribute.index === index) {
                return attribute;
            }
        }
    }

    function modifyStyleFunction(source) {
        // Replace occurrences of czm_tiles3d_style_DEFAULTPROPERTY
        var length = defaultProperties.length;
        for (var i = 0; i < length; ++i) {
            var property = defaultProperties[i];
            var styleName = 'czm_tiles3d_style_' + property;
            var replaceName = property.toLowerCase();
            source = source.replace(new RegExp(styleName + '(\\W)', 'g'), replaceName + '$1');
        }

        // Edit the function header to accept the point position, color, and normal
        return source.replace('()', '(vec3 position, vec3 position_absolute, vec4 color, vec3 normal)');
    }

    function createShaders(content, frameState, style) {
        var i;
        var name;
        var attribute;

        var context = frameState.context;
        var batchTable = content._batchTable;
        var hasBatchTable = defined(batchTable);
        var hasStyle = defined(style);
        var isQuantized = content._isQuantized;
        var isQuantizedDraco = content._isQuantizedDraco;
        var isOctEncoded16P = content._isOctEncoded16P;
        var isOctEncodedDraco = content._isOctEncodedDraco;
        var isRGB565 = content._isRGB565;
        var isTranslucent = content._isTranslucent;
        var hasColors = content._hasColors;
        var hasNormals = content._hasNormals;
        var hasBatchIds = content._hasBatchIds;
        var backFaceCulling = content._backFaceCulling;
        var vertexArray = content._drawCommand.vertexArray;
        var attenuation = content._attenuation;

        var colorStyleFunction;
        var showStyleFunction;
        var pointSizeStyleFunction;
        var styleTranslucent = isTranslucent;

        if (hasBatchTable) {
            // Styling is handled in the batch table
            hasStyle = false;
        }

        if (hasStyle) {
            var shaderState = {
                translucent : false
            };
            colorStyleFunction = style.getColorShaderFunction('getColorFromStyle', 'czm_tiles3d_style_', shaderState);
            showStyleFunction = style.getShowShaderFunction('getShowFromStyle', 'czm_tiles3d_style_', shaderState);
            pointSizeStyleFunction = style.getPointSizeShaderFunction('getPointSizeFromStyle', 'czm_tiles3d_style_', shaderState);
            if (defined(colorStyleFunction) && shaderState.translucent) {
                styleTranslucent = true;
            }
        }

        content._styleTranslucent = styleTranslucent;

        var hasColorStyle = defined(colorStyleFunction);
        var hasShowStyle = defined(showStyleFunction);
        var hasPointSizeStyle = defined(pointSizeStyleFunction);

        // Get the properties in use by the style
        var styleableProperties = [];

        if (hasColorStyle) {
            getStyleableProperties(colorStyleFunction, styleableProperties);
            colorStyleFunction = modifyStyleFunction(colorStyleFunction);
        }
        if (hasShowStyle) {
            getStyleableProperties(showStyleFunction, styleableProperties);
            showStyleFunction = modifyStyleFunction(showStyleFunction);
        }
        if (hasPointSizeStyle) {
            getStyleableProperties(pointSizeStyleFunction, styleableProperties);
            pointSizeStyleFunction = modifyStyleFunction(pointSizeStyleFunction);
        }

        var usesColorSemantic = styleableProperties.indexOf('COLOR') >= 0;
        var usesNormalSemantic = styleableProperties.indexOf('NORMAL') >= 0;

        // Split default properties from user properties
        var userProperties = styleableProperties.filter(function(property) { return defaultProperties.indexOf(property) === -1; });

        if (usesNormalSemantic && !hasNormals) {
            throw new RuntimeError('Style references the NORMAL semantic but the point cloud does not have normals');
        }

        // Disable vertex attributes that aren't used in the style, enable attributes that are
        var styleableShaderAttributes = content._styleableShaderAttributes;
        for (name in styleableShaderAttributes) {
            if (styleableShaderAttributes.hasOwnProperty(name)) {
                attribute = styleableShaderAttributes[name];
                var enabled = (userProperties.indexOf(name) >= 0);
                var vertexAttribute = getVertexAttribute(vertexArray, attribute.location);
                vertexAttribute.enabled = enabled;
            }
        }

        var usesColors = hasColors && (!hasColorStyle || usesColorSemantic);
        if (hasColors) {
            // Disable the color vertex attribute if the color style does not reference the color semantic
            var colorVertexAttribute = getVertexAttribute(vertexArray, colorLocation);
            colorVertexAttribute.enabled = usesColors;
        }

        var attributeLocations = {
            a_position : positionLocation
        };
        if (usesColors) {
            attributeLocations.a_color = colorLocation;
        }
        if (hasNormals) {
            attributeLocations.a_normal = normalLocation;
        }
        if (hasBatchIds) {
            attributeLocations.a_batchId = batchIdLocation;
        }

        var attributeDeclarations = '';

        var length = userProperties.length;
        for (i = 0; i < length; ++i) {
            name = userProperties[i];
            attribute = styleableShaderAttributes[name];
            if (!defined(attribute)) {
                throw new RuntimeError('Style references a property "' + name + '" that does not exist or is not styleable.');
            }

            var componentCount = attribute.componentCount;
            var attributeName = 'czm_tiles3d_style_' + name;
            var attributeType;
            if (componentCount === 1) {
                attributeType = 'float';
            } else {
                attributeType = 'vec' + componentCount;
            }

            attributeDeclarations += 'attribute ' + attributeType + ' ' + attributeName + '; \n';
            attributeLocations[attributeName] = attribute.location;
        }

        var vs = 'attribute vec3 a_position; \n' +
                 'varying vec4 v_color; \n' +
                 'uniform vec4 u_pointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier; \n' +
                 'uniform vec4 u_constantColor; \n' +
                 'uniform vec4 u_highlightColor; \n';
        vs += 'float u_pointSize; \n' +
              'float u_tilesetTime; \n';

        if (attenuation) {
            vs += 'float u_geometricError; \n' +
                  'float u_depthMultiplier; \n';
        }

        vs += attributeDeclarations;

        if (usesColors) {
            if (isTranslucent) {
                vs += 'attribute vec4 a_color; \n';
            } else if (isRGB565) {
                vs += 'attribute float a_color; \n' +
                      'const float SHIFT_RIGHT_11 = 1.0 / 2048.0; \n' +
                      'const float SHIFT_RIGHT_5 = 1.0 / 32.0; \n' +
                      'const float SHIFT_LEFT_11 = 2048.0; \n' +
                      'const float SHIFT_LEFT_5 = 32.0; \n' +
                      'const float NORMALIZE_6 = 1.0 / 64.0; \n' +
                      'const float NORMALIZE_5 = 1.0 / 32.0; \n';
            } else {
                vs += 'attribute vec3 a_color; \n';
            }
        }
        if (hasNormals) {
            if (isOctEncoded16P || isOctEncodedDraco) {
                vs += 'attribute vec2 a_normal; \n';
            } else {
                vs += 'attribute vec3 a_normal; \n';
            }
        }

        if (hasBatchIds) {
            vs += 'attribute float a_batchId; \n';
        }

        if (isQuantized || isQuantizedDraco || isOctEncodedDraco) {
            vs += 'uniform vec4 u_quantizedVolumeScaleAndOctEncodedRange; \n';
        }

        if (hasColorStyle) {
            vs += colorStyleFunction;
        }

        if (hasShowStyle) {
            vs += showStyleFunction;
        }

        if (hasPointSizeStyle) {
            vs += pointSizeStyleFunction;
        }

        vs += 'void main() \n' +
              '{ \n' +
              '    u_pointSize = u_pointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier.x; \n' +
              '    u_tilesetTime = u_pointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier.y; \n';

        if (attenuation) {
            vs += '    u_geometricError = u_pointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier.z; \n' +
                  '    u_depthMultiplier = u_pointSizeAndTilesetTimeAndGeometricErrorAndDepthMultiplier.w; \n';
        }

        if (usesColors) {
            if (isTranslucent) {
                vs += '    vec4 color = a_color; \n';
            } else if (isRGB565) {
                vs += '    float compressed = a_color; \n' +
                      '    float r = floor(compressed * SHIFT_RIGHT_11); \n' +
                      '    compressed -= r * SHIFT_LEFT_11; \n' +
                      '    float g = floor(compressed * SHIFT_RIGHT_5); \n' +
                      '    compressed -= g * SHIFT_LEFT_5; \n' +
                      '    float b = compressed; \n' +
                      '    vec3 rgb = vec3(r * NORMALIZE_5, g * NORMALIZE_6, b * NORMALIZE_5); \n' +
                      '    vec4 color = vec4(rgb, 1.0); \n';
            } else {
                vs += '    vec4 color = vec4(a_color, 1.0); \n';
            }
        } else {
            vs += '    vec4 color = u_constantColor; \n';
        }

        if (isQuantized || isQuantizedDraco) {
            vs += '    vec3 position = a_position * u_quantizedVolumeScaleAndOctEncodedRange.xyz; \n';
        } else {
            vs += '    vec3 position = a_position; \n';
        }
        vs += '    vec3 position_absolute = vec3(czm_model * vec4(position, 1.0)); \n';

        if (hasNormals) {
            if (isOctEncoded16P) {
                vs += '    vec3 normal = czm_octDecode(a_normal); \n';
            } else if (isOctEncodedDraco) {
                // Draco oct-encoding decodes to zxy order
                vs += '    vec3 normal = czm_octDecode(a_normal, u_quantizedVolumeScaleAndOctEncodedRange.w).zxy; \n';
            } else {
                vs += '    vec3 normal = a_normal; \n';
            }
        } else {
            vs += '    vec3 normal = vec3(1.0); \n';
        }

        if (hasColorStyle) {
            vs += '    color = getColorFromStyle(position, position_absolute, color, normal); \n';
        }

        if (hasShowStyle) {
            vs += '    float show = float(getShowFromStyle(position, position_absolute, color, normal)); \n';
        }

        if (hasPointSizeStyle) {
            vs += '    gl_PointSize = getPointSizeFromStyle(position, position_absolute, color, normal); \n';
        } else if (attenuation) {
            vs += '    vec4 positionEC = czm_modelView * vec4(position, 1.0); \n' +
                  '    float depth = -positionEC.z; \n' +
                  // compute SSE for this point
                  '    gl_PointSize = min((u_geometricError / depth) * u_depthMultiplier, u_pointSize); \n';
        } else {
            vs += '    gl_PointSize = u_pointSize; \n';
        }

        vs += '    color = color * u_highlightColor; \n';

        if (hasNormals) {
            vs += '    normal = czm_normal * normal; \n' +
                  '    float diffuseStrength = czm_getLambertDiffuse(czm_sunDirectionEC, normal); \n' +
                  '    diffuseStrength = max(diffuseStrength, 0.4); \n' + // Apply some ambient lighting
                  '    color.xyz *= diffuseStrength; \n';
        }

        vs += '    v_color = color; \n' +
              '    gl_Position = czm_modelViewProjection * vec4(position, 1.0); \n';

        if (hasNormals && backFaceCulling) {
            vs += '    float visible = step(-normal.z, 0.0); \n' +
                  '    gl_Position *= visible; \n' +
                  '    gl_PointSize *= visible; \n';
        }

        if (hasShowStyle) {
            vs += '    gl_Position *= show; \n' +
                  '    gl_PointSize *= show; \n';
        }

        vs += '} \n';

        var fs = 'varying vec4 v_color; \n';

        fs +=  'void main() \n' +
               '{ \n' +
               '    gl_FragColor = v_color; \n';

        fs += '} \n';

        var drawVS = vs;
        var drawFS = fs;

        if (hasBatchTable) {
            // Batched points always use the HIGHLIGHT color blend mode
            drawVS = batchTable.getVertexShaderCallback(false, 'a_batchId', undefined)(drawVS);
            drawFS = batchTable.getFragmentShaderCallback(false, undefined)(drawFS);
        }

        var pickVS = vs;
        var pickFS = fs;

        if (hasBatchTable) {
            pickVS = batchTable.getPickVertexShaderCallback('a_batchId')(pickVS);
            pickFS = batchTable.getPickFragmentShaderCallback()(pickFS);
        } else {
            pickFS = ShaderSource.createPickFragmentShaderSource(pickFS, 'uniform');
        }

        var drawCommand = content._drawCommand;
        if (defined(drawCommand.shaderProgram)) {
            // Destroy the old shader
            drawCommand.shaderProgram.destroy();
        }
        drawCommand.shaderProgram = ShaderProgram.fromCache({
            context : context,
            vertexShaderSource : drawVS,
            fragmentShaderSource : drawFS,
            attributeLocations : attributeLocations
        });

        var pickCommand = content._pickCommand;
        if (defined(pickCommand.shaderProgram)) {
            // Destroy the old shader
            pickCommand.shaderProgram.destroy();
        }
        pickCommand.shaderProgram = ShaderProgram.fromCache({
            context : context,
            vertexShaderSource : pickVS,
            fragmentShaderSource : pickFS,
            attributeLocations : attributeLocations
        });

        try {
            // Check if the shader compiles correctly. If not there is likely a syntax error with the style.
            drawCommand.shaderProgram._bind();
        } catch (error) {
            // Rephrase the error.
            throw new RuntimeError('Error generating style shader: this may be caused by a type mismatch, index out-of-bounds, or other syntax error.');
        }
    }

    var scratchComputedTranslation = new Cartesian4();
    var scratchComputedMatrixIn2D = new Matrix4();

    var maxDecodingConcurrency = Math.max(FeatureDetection.hardwareConcurrency - 1, 1);
    var decoderTaskProcessor;
    function getDecoderTaskProcessor() {
        if (!defined(decoderTaskProcessor)) {
            decoderTaskProcessor = new TaskProcessor('decodeDracoPointCloud', maxDecodingConcurrency, {
                modulePath : 'ThirdParty/draco_wasm_wrapper.js',
                wasmBinaryFile : 'ThirdParty/draco_decoder.wasm',
                fallbackModulePath : 'ThirdParty/draco_decoder.js'
            });
        }

        return decoderTaskProcessor;
    }

    function runDecoderTaskProcessor(draco) {
        if (FeatureDetection.isInternetExplorer()) {
            return when.reject(new RuntimeError('Draco decoding is not currently supported in Internet Explorer.'));
        }

        var decoderTaskProcessor = getDecoderTaskProcessor();
        var promise = decoderTaskProcessor.scheduleTask(draco, [draco.buffer.buffer]);
        if (!defined(promise)) {
            return;
        }

        return promise;
    }


    function decodeDraco(content, context) {
        if (content._decodingState === DecodingState.READY) {
            return false;
        }
        if (content._decodingState === DecodingState.NEEDS_DECODE) {
            var parsedContent = content._parsedContent;
            var draco = parsedContent.draco;
            var decodePromise = runDecoderTaskProcessor(draco, context);
            if (defined(decodePromise)) {
                content._decodingState = DecodingState.DECODING;
                decodePromise.then(function(result) {
                    content._decodingState = DecodingState.READY;
                    var decodedPositions = defined(result.POSITION) ? result.POSITION.buffer : undefined;
                    var decodedRgb = defined(result.RGB) ? result.RGB.buffer : undefined;
                    var decodedRgba = defined(result.RGBA) ? result.RGBA.buffer : undefined;
                    var decodedNormals = defined(result.NORMAL) ? result.NORMAL.buffer : undefined;
                    var decodedBatchIds = defined(result.BATCH_ID) ? result.BATCH_ID.buffer : undefined;
                    parsedContent.positions = defaultValue(decodedPositions, parsedContent.positions);
                    parsedContent.colors = defaultValue(defaultValue(decodedRgba, decodedRgb), parsedContent.colors);
                    parsedContent.normals = defaultValue(decodedNormals, parsedContent.normals);
                    parsedContent.batchIds = defaultValue(decodedBatchIds, parsedContent.batchIds);
                    if (content._isQuantizedDraco) {
                        var quantization = result.POSITION.quantization;
                        var scale = quantization.range / (1 << quantization.quantizationBits);
                        content._quantizedVolumeScale = Cartesian3.fromElements(scale, scale, scale);
                        content._quantizedVolumeOffset = Cartesian3.unpack(quantization.minValues);
                    }
                    if (content._isOctEncodedDraco) {
                        content._octEncodedRange = (1 << result.NORMAL.quantization.quantizationBits) - 1.0;
                    }
                }).otherwise(function(error) {
                    content._decodingState = DecodingState.FAILED;
                    content._readyPromise.reject(error);
                });
            }
        }
        return true;
    }

    /**
     * @inheritdoc Cesium3DTileContent#update
     */
    PointCloudFrame.prototype.update = function(frameState) {
        var context = frameState.context;
        var decoding = decodeDraco(this, context);
        if (decoding) {
            return;
        }

        var modelMatrix = this.modelMatrix;
        var modelMatrixChanged = !Matrix4.equals(this._modelMatrix, modelMatrix);
        var updateModelMatrix = modelMatrixChanged || this._mode !== frameState.mode;
        var style = this._stream.style;

        if (!defined(this._drawCommand)) {
            createResources(this, frameState);
            createShaders(this, frameState, style);
            updateModelMatrix = true;
            this._parsedContent = undefined; // Unload
        }

        // Update attenuation
        var pointCloudShading = this._stream.pointCloudShading;
        if (defined(pointCloudShading)) {
            var formerAttenuation = this._attenuation;
            this._attenuation = pointCloudShading.attenuation;
            this._geometricErrorScale = pointCloudShading.geometricErrorScale;
            this._maximumAttenuation = pointCloudShading.maximumAttenuation;
            this._baseResolution = pointCloudShading.baseResolution;
            if (this._attenuation !== formerAttenuation) {
                createShaders(this, frameState, style);
            }
        }

        if (style !== this._style) {
            this._style = style;
            createShaders(this, frameState, style);
        }

        if (updateModelMatrix) {
            Matrix4.clone(modelMatrix, this._modelMatrix);
            if (defined(this._rtcCenter)) {
                Matrix4.multiplyByTranslation(modelMatrix, this._rtcCenter, this._drawCommand.modelMatrix);
            } else if (defined(this._quantizedVolumeOffset)) {
                Matrix4.multiplyByTranslation(modelMatrix, this._quantizedVolumeOffset, this._drawCommand.modelMatrix);
            } else {
                Matrix4.clone(modelMatrix, this._drawCommand.modelMatrix);
            }

            Matrix4.clone(this._drawCommand.modelMatrix, this._pickCommand.modelMatrix);
        }

        // Update the render state
        var isTranslucent = (this._highlightColor.alpha < 1.0) || (this._constantColor.alpha < 1.0) || this._styleTranslucent;
        this._drawCommand.renderState = isTranslucent ? this._translucentRenderState : this._opaqueRenderState;
        this._drawCommand.pass = isTranslucent ? Pass.TRANSLUCENT : Pass.CESIUM_3D_TILE;

        var commandList = frameState.commandList;

        var passes = frameState.passes;
        if (passes.render) {
            commandList.push(this._drawCommand);
        }
        if (passes.pick) {
            commandList.push(this._pickCommand);
        }
    };

    return PointCloudStream;
});
