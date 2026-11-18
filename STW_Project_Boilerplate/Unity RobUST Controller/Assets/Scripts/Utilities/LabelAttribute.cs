using UnityEngine;

// Sets a field's Inspector label exactly, since Unity's automatic labels split "CoM" into "Co M" and show "Ee"/"Com".
// Also honours [Min] on the same field.
public class LabelAttribute : PropertyAttribute
{
    public readonly string Text;
    public LabelAttribute(string text) => Text = text;
}

#if UNITY_EDITOR
[UnityEditor.CustomPropertyDrawer(typeof(LabelAttribute))]
public class LabelDrawer : UnityEditor.PropertyDrawer
{
    public override float GetPropertyHeight(UnityEditor.SerializedProperty property, GUIContent label) =>
        UnityEditor.EditorGUI.GetPropertyHeight(property, label, true);

    public override void OnGUI(Rect position, UnityEditor.SerializedProperty property, GUIContent label)
    {
        label.text = ((LabelAttribute)attribute).Text;
        UnityEditor.EditorGUI.PropertyField(position, property, label, true);
        if (property.propertyType == UnityEditor.SerializedPropertyType.Float
            && System.Attribute.GetCustomAttribute(fieldInfo, typeof(MinAttribute)) is MinAttribute min)
            property.floatValue = Mathf.Max(property.floatValue, min.min);
    }
}
#endif
