# Call Service Panel
This is an example panel to call services. Base a new extension off of this panel if it requires calling services.

## Usage
Enter the Service Name and Request in the appropriate input fields.

Ensure that the request is formatted as a [JavaScript object](https://developer.mozilla.org/en-US/docs/Learn/JavaScript/Objects/JSON#json_structure) and conforms to the service message.

## Example
Service Name: `/controls/enable`

Request (`SetBool.srv`):
```js
{
    "data": true
}
```

**Note:** To start the `/controls/enable` service, run:
```bash
ros2 launch controls controls.launch
```