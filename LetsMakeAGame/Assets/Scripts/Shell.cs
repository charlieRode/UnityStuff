using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Shell : MonoBehaviour {

    public Rigidbody rigidBody;
    public Transform ejectDirectionTarget;
    Vector3 ejectDirection;
    public Vector2 ejectForceRange;

    float lifeTime = 4;
    float fadeTime = 2;

    void Start()
    {
        ejectDirection = (ejectDirectionTarget.position - transform.position).normalized;
        float force = Random.Range(ejectForceRange.x, ejectForceRange.y);
        rigidBody = GetComponent<Rigidbody>();
        rigidBody.AddForce(ejectDirection * force);
        rigidBody.AddTorque(Random.insideUnitSphere);

        StartCoroutine(Fade());
    }

    IEnumerator Fade()
    {
        yield return new WaitForSeconds(lifeTime);

        float pct = 0;
        float fadeSpeed = 1 / fadeTime;
        Material mat = GetComponent<Renderer>().material;
        Color initialColor = mat.color;

        while (pct < 1)
        {
            pct += Time.deltaTime * fadeSpeed;
            Color.Lerp(initialColor, Color.clear, pct);
            yield return null;
        }

        Destroy(this.gameObject);
    }
}
