package com.samusynth.questcamera.json

import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import com.samusynth.questcamera.core.CameraMetadata

fun CameraMetadata.toJson(pretty: Boolean = false): String {
    val json = if (pretty) {
        Json { prettyPrint = true }
    } else {
        Json
    }
    return json.encodeToString(this)
}

