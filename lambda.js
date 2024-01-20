const AWS = require('aws-sdk');
AWS.config.update( {
  region: 'us-west-2'
});

const dynamodb = new AWS.DynamoDB.DocumentClient();
const dynamoTableName ='humedad-tabla';
const sensorPath ='/sensorapi';

exports.handler = async function(event) {
    console.log('Request event: ', event);
    let response;
    switch (true) {
        case event.httpMethod === 'GET' && event.path === sensorPath:
            response = await getRegisters(event.queryStringParameters.startregister, event.queryStringParameters.maxregisters);
            break;
        case event.type === 'mqtt':
            const lastId = await return_last_id();
            const mqttStoreItem = {
                RegistroId: lastId + 1,
                Fecha: event.Fecha,
                Hora: event.Hora,
                medida: event.medida,
                comentario: event.comentario
            };
            response = await saveRegister(mqttStoreItem);
            break;
        default:
            response = buildResponse(404, { message: 'Not Found' });
    } 
    return response;
}

async function return_last_id(){
    const params = {
                TableName:dynamoTableName
            }
    const lastRegistroId = { value: 0 };
    await return_last_id_recursive(params, [], lastRegistroId);
    return lastRegistroId.value;
}

async function return_last_id_recursive(scanParams, itemArray, lastRegistroId) {
  try {
    const dynamoData = await dynamodb.scan(scanParams).promise();
    itemArray = itemArray.concat(dynamoData.Items);

    if (dynamoData.LastEvaluateKey) {
      scanParams.ExclusiveStartKey = dynamoData.LastEvaluateKey;
      return await return_last_id_recursive(scanParams, itemArray, lastRegistroId);
    } else {
      const sortedArray = itemArray.sort((a, b) => {
        if (a.RegistroId < b.RegistroId) {
          return -1;
        } else if (a.RegistroId > b.RegistroId) {
          return 1;
        } else {
          return 0;
        }
      });
      const tmpArray = sortedArray.reverse();
      lastRegistroId.value = tmpArray.length > 0 ? tmpArray[0].RegistroId : 0;
    }
  } catch (error) {
    console.error('error on returning ', error);
  }
}

function buildResponse(statusCode, body){
    return{
        statusCode: statusCode,
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(body)
    }
}


async function getRegisters(startregister,maxregisters){
    const params = {
        TableName:dynamoTableName
    }
    const lastRegistroId = { value: 0 };
    const allRegs = await scanDynamoRecords(params,[],startregister,maxregisters, lastRegistroId);
    const body = {
        registers: allRegs,
        start:parseInt(startregister),
        max:parseInt(maxregisters),
        lastRegistroId: lastRegistroId.value,
    }
    return buildResponse(200,body);
}

async function scanDynamoRecords(scanParams, itemArray, startregister, maxregisters, lastRegistroId) {
  try {
    const dynamoData = await dynamodb.scan(scanParams).promise();
    itemArray = itemArray.concat(dynamoData.Items);

    if (dynamoData.LastEvaluateKey) {
      scanParams.ExclusiveStartKey = dynamoData.LastEvaluateKey;
      return await scanDynamoRecords(scanParams, itemArray, startregister, maxregisters, lastRegistroId);
    } else {
      const sortedArray = itemArray.sort((a, b) => {
        if (a.RegistroId < b.RegistroId) {
          return -1;
        } else if (a.RegistroId > b.RegistroId) {
          return 1;
        } else {
          return 0;
        }
      });
      
      const tmpArray = sortedArray.reverse();
      
      lastRegistroId.value = tmpArray.length > 0 ? tmpArray[0].RegistroId : 0;

      const startIndex = startregister - 1;

      const resultArray = tmpArray.slice(startIndex);

      return resultArray.slice(0, maxregisters);
    }
  } catch (error) {
    console.error('error on returning ', error);
  }
}

async function saveRegister(requestbody){
    // Función asincrónica para verificar si un registro con el mismo RegistroId existe
    async function checkAndIncrement(registroId, maxAttempts) {
        if (maxAttempts <= 0) {
            throw new Error('Exceeded maximum attempts to find unique RegistroId');
        }

        const queryParams = {
            TableName: dynamoTableName,
            Key: {
                "RegistroId": registroId
            }
        };

        try {
            const existingRecord = await dynamodb.get(queryParams).promise();
            return existingRecord.Item
                ? await checkAndIncrement(registroId + 1, maxAttempts - 1)
                : registroId;
        } catch (error) {
            console.error('Error on checking record existence', error);
            throw error;
        }
    }
    try {
        const maxAttempts = 10;
        const uniqueRegistroId = await checkAndIncrement(requestbody.RegistroId, maxAttempts);
        const params = {
            TableName: dynamoTableName,
            Item: {
                ...requestbody,
                RegistroId: uniqueRegistroId
            },
            ConditionExpression: "attribute_not_exists(RegistroId)",  // Evitar duplicados
        };
        await dynamodb.put(params).promise();
        const body = {
            Operation: 'SAVE',
            Message: 'Success',
            Item: requestbody
        };
        return buildResponse(200, body);
    }
    catch (error){
        console.error('Error on saving ', error);
    }
    
}
