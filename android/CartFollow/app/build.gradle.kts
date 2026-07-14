plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.cartfollow"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.cartfollow"
        minSdk = 26
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    buildFeatures {
        viewBinding = true
    }

    // Android 15 (16 KB page size): ship uncompressed, 16 KB-aligned .so files.
    // AGP 8.5.1+ zip-aligns them on a 16 KB boundary when not legacy-packaged.
    packaging {
        jniLibs {
            useLegacyPackaging = false
        }
    }
}

dependencies {
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("com.google.android.material:material:1.12.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    implementation("androidx.lifecycle:lifecycle-service:2.8.4")

    // CameraX 1.4.2+ ships the 16 KB-aligned libimage_processing_util_jni.so.
    // Older versions (1.3.x) are the usual cause of "ELF alignment check failed".
    implementation("androidx.camera:camera-core:1.4.2")
    implementation("androidx.camera:camera-camera2:1.4.2")
    implementation("androidx.camera:camera-lifecycle:1.4.2")
    implementation("androidx.camera:camera-view:1.4.2")

    implementation("com.google.mlkit:pose-detection:18.0.0-beta5")
}
