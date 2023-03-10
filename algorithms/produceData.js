let mock  = [ ['title1', 'title2', 'title3'], ['ignore', 'ignore', 'ignore'], [1, 2, 3], [1, 2, 3], [1, 2, 3], [1, 2, 3], [1, 2, 3] ]

function extractKeys (fields=['title1', 'title3'], data=mock) {

    let transformed = []
    let size = data.length
    if (size <= 2) return transformed

    let fieldIndexes = fields.map(field=>data[0].indexOf(field)).filter(field=>field!=-1)

    for (let k = 2; k < size; k++) {
        // {}
        let row = fieldIndexes.reduce( (global_, currentIndex)=>{
            global_[ data[0][currentIndex] ] = data[k][currentIndex]
            return global_
        }, {} )

        transformed.push(row)

    }

    return transformed

}

let data = extractKeys([ 'title1', 'title2' ], mock)


data:extractKeys([ 'key1', 'key2', 'key3' ], data)


data