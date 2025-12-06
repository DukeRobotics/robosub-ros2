# Publish Topic Panel
This panel allows you to publish topics. Compared to the build-in Foxglove Publish panel, this custom panel allows for the following:
- Publishing custom Duke Robotics Club message schemas.
- Publishing at continuous rates.

## Usage
Enter the Topic Name, Schema Name, and Request in the appropriate input fields.

Ensure that the request is formatted as a [JavaScript object](https://developer.mozilla.org/en-US/docs/Learn/JavaScript/Objects/JSON#json_structure) and conforms to the selected schema.

## Example
Topic Name: `/example_topic`

Schema Name: `std_msgs/String`

Request:
```js
{
  "data": "Example data"
}
```