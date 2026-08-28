# JavaScript calls this method through WebView.addJavascriptInterface.
-keepclassmembers class com.dog.nav2controller.Nav2JavascriptBridge {
    @android.webkit.JavascriptInterface <methods>;
}
