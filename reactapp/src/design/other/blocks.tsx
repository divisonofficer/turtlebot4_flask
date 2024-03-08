const VDivBlock = (props: { color?: string }) => {
    return (
        <div style={{
            width: '100%',
            height: '8px',
            borderRadius: '4px',
            background: props.color || '#B1E3FF'
        }}></div>
    )
}

export {
    VDivBlock
}