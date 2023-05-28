function navbar_handle(class_name, url) {
    $(class_name).click(() => {
        $.ajax({
            type: "get",
            url: url,
            context: document.body
        }).done(function (resp) {
            $("#content").html(resp);
        });
    })
}