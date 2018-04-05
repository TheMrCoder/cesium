define([
        '../Core/ComponentDatatype',
        '../Core/defined',
        '../Core/RuntimeError',
        './createTaskProcessorWorker'
    ], function(
        ComponentDatatype,
        defined,
        RuntimeError,
        createTaskProcessorWorker) {
    'use strict';

    var draco;
    var dracoDecoder;

    function getAttributeTypeFromSemantic(draco, semantic) {
        switch (semantic) {
            case 'POSITION':
                return draco.POSITION;
            case 'NORMAL':
                return draco.NORMAL;
            case 'RGB':
            case 'RGBA':
                return draco.COLOR;
            case 'BATCH_ID':
                return draco.GENERIC;
        }
    }

    function getComponentDatatypeFromAttributeDataType(attributeDataType) {
        // Some attribute types are casted down to 32 bit since Draco only returns 32 bit values
        switch (attributeDataType) {
            case 1: return ComponentDatatype.BYTE;           // DT_INT8
            case 2: return ComponentDatatype.UNSIGNED_BYTE;  // DT_UINT8
            case 3: return ComponentDatatype.SHORT;          // DT_INT16
            case 4: return ComponentDatatype.UNSIGNED_SHORT; // DT_UINT16
            case 5: return ComponentDatatype.INT;            // DT_INT32
            case 6: return ComponentDatatype.UNSIGNED_INT;   // DT_UINT32
            case 7: return ComponentDatatype.INT;            // DT_INT64
            case 8: return ComponentDatatype.UNSIGNED_INT;   // DT_UINT64
            case 9: return ComponentDatatype.FLOAT;          // DT_FLOAT32
            case 10: return ComponentDatatype.FLOAT;         // DT_FLOAT64
            case 11: return ComponentDatatype.BYTE;          // DT_BOOL
        }
    }

    function decodeDracoPointCloud(parameters) {
        var dequantizeInShader = parameters.dequantizeInShader;
        var results = {};

        if (!defined(dracoDecoder)) {
            draco = self.wasmModule;
            dracoDecoder = new draco.Decoder();
        }

        if (dequantizeInShader) {
            // TODO : for some reason colors don't work when these lines are called
            dracoDecoder.SkipAttributeTransform(draco.POSITION);
            dracoDecoder.SkipAttributeTransform(draco.NORMAL);
        }

        var buffer = new draco.DecoderBuffer();
        buffer.Init(parameters.buffer, parameters.buffer.length);

        var geometryType = dracoDecoder.GetEncodedGeometryType(buffer);
        if (geometryType !== draco.POINT_CLOUD) {
            throw new RuntimeError('Draco geometry type must be POINT_CLOUD.');
        }

        var dracoPointCloud = new draco.PointCloud();
        var decodingStatus = dracoDecoder.DecodeBufferToPointCloud(buffer, dracoPointCloud);
        if (!decodingStatus.ok() || dracoPointCloud.ptr === 0) {
            throw new RuntimeError('Error decoding draco point cloud: ' + decodingStatus.error_msg());
        }

        draco.destroy(buffer);

        var numPoints = dracoPointCloud.num_points();

        var semantics = parameters.semantics;
        var semanticsLength = semantics.length;
        for (var i = 0; i < semanticsLength; ++i) {
            var semantic = semantics[i];
            var attributeType = getAttributeTypeFromSemantic(draco, semantic);
            if (!defined(attributeType)) {
                throw new RuntimeError('Error decoding draco point cloud: ' + semantic + ' is not a valid draco semantic');
            }
            var attributeId = dracoDecoder.GetAttributeId(dracoPointCloud, attributeType);
            var attribute = dracoDecoder.GetAttribute(dracoPointCloud, attributeId);
            var quantize = dequantizeInShader && (semantic === 'POSITION');
            var octEncode = dequantizeInShader && (semantic === 'NORMAL');
            var numComponents = attribute.num_components();

            /*eslint-disable no-undef-init*/
            var quantization = undefined;
            var transform;
            var attributeData;

            if (quantize) {
                transform = new draco.AttributeQuantizationTransform();
                transform.InitFromAttribute(attribute);
                var minValues = new Array(numComponents);
                for (var j = 0; j < numComponents; ++j) {
                    minValues[j] = transform.min_value(j);
                }
                quantization = {
                    quantizationBits : transform.quantization_bits(),
                    minValues : minValues,
                    range : transform.range()
                };
                draco.destroy(transform);
            } else if (octEncode) {
                // TODO : check that num_components is actually 2 and stuff here
                transform = new draco.AttributeOctahedronTransform();
                transform.InitFromAttribute(attribute);
                quantization = {
                    quantizationBits : transform.quantization_bits()
                };
                draco.destroy(transform);
            }

            var vertexArrayLength = numPoints * numComponents;

            var attributeDataType = attribute.data_type();
            var componentDatatype = getComponentDatatypeFromAttributeDataType(attributeDataType);

            if (quantize) {
                componentDatatype = ComponentDatatype.UNSIGNED_SHORT;
            } else if (octEncode) {
                componentDatatype = ComponentDatatype.SHORT;
            }

            if (!defined(componentDatatype)) {
                throw new RuntimeError('Error decoding draco point cloud: ' + attributeDataType + ' is not a valid draco attribute data type');
            }

            if (componentDatatype === ComponentDatatype.FLOAT) {
                attributeData = new draco.DracoFloat32Array();
                dracoDecoder.GetAttributeFloatForAllPoints(dracoPointCloud, attribute, attributeData);
            } else {
                attributeData = new draco.DracoInt32Array();
                dracoDecoder.GetAttributeInt32ForAllPoints(dracoPointCloud, attribute, attributeData);
            }

            var vertexArray = ComponentDatatype.createTypedArray(componentDatatype, vertexArrayLength);
            for (var k = 0; k < vertexArrayLength; ++k) {
                vertexArray[k] = attributeData.GetValue(k);
            }

            draco.destroy(attributeData);

            results[semantic] = {
                buffer : vertexArray,
                quantization : quantization
            };
        }

        draco.destroy(dracoPointCloud);
        return results;
    }

    return createTaskProcessorWorker(decodeDracoPointCloud);
});
